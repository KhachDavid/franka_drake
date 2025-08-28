#include "franka_drake/simulator_server.h"
#include "franka_drake/franka_drake_module.h"
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/TCPServer.h>
#include <Poco/Net/TCPServerConnection.h>
#include <Poco/Net/TCPServerConnectionFactory.h>
#include <Poco/Thread.h>
#include <atomic>
#include <array>
#include <mutex>
#include <thread>
#include <iostream>
#include <sstream>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/analysis/runge_kutta5_integrator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/geometry/meshcat.h>
#include <Eigen/Core>

namespace franka_drake {

namespace {
// Shared torque buffer for network thread <-> sim thread
std::array<double, 7> g_tau{};
std::atomic<bool> g_stop{false};
std::mutex g_tau_mutex;

class TorqueConnectionHandler : public Poco::Net::TCPServerConnection {
 public:
  TorqueConnectionHandler(const Poco::Net::StreamSocket& s) : Poco::Net::TCPServerConnection(s) {}
  void run() override {
    Poco::Net::StreamSocket& ss = socket();
    try {
      std::string buffer;
      buffer.reserve(256);
      char tmp[256];
      while (true) {
        int n = ss.receiveBytes(tmp, sizeof(tmp));
        if (n <= 0) break;
        buffer.append(tmp, n);
        std::size_t pos;
        while ((pos = buffer.find('\n')) != std::string::npos) {
          std::string line = buffer.substr(0, pos);
          buffer.erase(0, pos + 1);
          if (line == "STOP") {
            g_stop.store(true);
            std::cout << "Received STOP signal" << std::endl;
            continue;
          }
          std::istringstream iss(line);
          double t;
          std::array<double, 7> tau{};
          if (!(iss >> t)) continue;
          for (int i = 0; i < 7 && iss; ++i) {
            iss >> tau[i];
          }
          {
            std::lock_guard<std::mutex> lock(g_tau_mutex);
            g_tau = tau;
          }
        }
      }
    } catch (Poco::Exception& exc) {
      std::cerr << "Poco network error: " << exc.displayText() << std::endl;
    }
  }
};

class TorqueConnectionFactory : public Poco::Net::TCPServerConnectionFactory {
 public:
  Poco::Net::TCPServerConnection* createConnection(const Poco::Net::StreamSocket& socket) override {
    return new TorqueConnectionHandler(socket);
  }
};
} // namespace

SimulatorServer::SimulatorServer(int port, const std::string& package_xml, const std::string& urdf_path, double time_step)
    : port_(port), package_xml_(package_xml), urdf_path_(urdf_path), time_step_(time_step) {
  builder_ = std::make_unique<drake::systems::DiagramBuilder<double>>();
  module_ = std::make_unique<FrankaDrakeModule>(builder_.get(), package_xml_, urdf_path_, time_step_);
}

void SimulatorServer::Run() {
  // Start TCP server in a background thread
  Poco::Net::ServerSocket svs(port_);
  Poco::Net::TCPServer srv(new TorqueConnectionFactory(), svs);
  srv.start();
  std::cout << "TCP server started on port " << port_ << std::endl;

  // Drake simulation setup
  auto* plant = module_->get_plant();
  // Add gravity compensation
  using drake::systems::controllers::InverseDynamics;
  auto* g_comp = builder_->AddSystem<InverseDynamics<double>>(
      plant, InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);
  builder_->Connect(plant->get_state_output_port(), g_comp->get_input_port_estimated_state());

  // Network torque system
  class NetworkTorqueSystem final : public drake::systems::LeafSystem<double> {
   public:
    NetworkTorqueSystem() {
      this->DeclareVectorOutputPort("tau_vec", drake::systems::BasicVector<double>(7),
                                   &NetworkTorqueSystem::DoCalcOutput);
    }
   private:
    void DoCalcOutput(const drake::systems::Context<double>&,
                      drake::systems::BasicVector<double>* out) const {
      std::array<double, 7> local;
      {
        std::lock_guard<std::mutex> lock(g_tau_mutex);
        local = g_tau;
      }
      Eigen::VectorXd v(7);
      for (int i = 0; i < 7; ++i) v[i] = local[i];
      out->SetFromVector(v);
    }
  };
  auto* tau_from_network = builder_->AddSystem<NetworkTorqueSystem>();

  // Sum gravity comp + network torque
  int n = plant->num_positions();
  auto* adder = builder_->AddSystem<drake::systems::Adder<double>>(2, n);
  builder_->Connect(g_comp->get_output_port(), adder->get_input_port(0));
  builder_->Connect(tau_from_network->get_output_port(), adder->get_input_port(1));
  builder_->Connect(adder->get_output_port(), plant->get_actuation_input_port());

  // Visualization
  drake::geometry::MeshcatParams params;
  params.host = "0.0.0.0";      // listen on all interfaces
  params.port = std::nullopt;   // keep previous behavior; set a port if you want a fixed one
  auto meshcat = std::make_shared<drake::geometry::Meshcat>(params);
  drake::visualization::AddDefaultVisualization(builder_.get(), meshcat);

  // Logging
  int n_state = plant->num_positions() + plant->num_velocities();
  auto* logger = builder_->AddSystem<drake::systems::VectorLogSink<double>>(n_state);
  builder_->Connect(plant->get_state_output_port(), logger->get_input_port());
  auto* torque_logger = builder_->AddSystem<drake::systems::VectorLogSink<double>>(plant->num_actuators());
  builder_->Connect(adder->get_output_port(), torque_logger->get_input_port());

  // Build diagram and simulate
  auto diagram = builder_->Build();
  drake::systems::Simulator<double> sim(*diagram);
  sim.set_target_realtime_rate(1.0);
  auto& rk = sim.reset_integrator<drake::systems::RungeKutta5Integrator<double>>();
  rk.set_target_accuracy(1e-12);
  rk.set_maximum_step_size(1e-4);

  // Set initial state
  Eigen::VectorXd q_des(plant->num_positions()), v_des(plant->num_velocities());
  q_des << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/2;
  v_des.setZero();
  auto& root = sim.get_mutable_context();
  auto& plant_ctx = plant->GetMyMutableContextFromRoot(&root);
  plant->SetPositions(&plant_ctx, q_des);
  plant->SetVelocities(&plant_ctx, v_des);

  sim.Initialize();
  std::cout << "Running ... press <Enter> to begin"; std::cin.get();

  const double step = 0.02;
  // double target = 0.0;
  // while (!g_stop.load() && target < 120.0) {
  while (!g_stop.load()) {
    // target += step;
    sim.AdvanceTo(sim.get_context().get_time() + step);
  }
  srv.stop();
  std::cout << "Simulation stopped." << std::endl;
}

} // namespace franka_drake 