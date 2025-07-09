#include <chrono>
#include <iostream>
#include <thread>

// Drake core / geometry / multibody
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>

// Drake systems
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/analysis/runge_kutta5_integrator.h>
#include <drake/systems/analysis/simulator_config.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/controllers/pid_controller.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/vector_log_sink.h>

// Drake visualization
#include <drake/visualization/visualization_config_functions.h>

// Eigen helpers
#include <Eigen/Core>

#include <drake/math/rigid_transform.h>

#include <drake/geometry/meshcat.h>
#include <memory>
#include <atomic>
#include <array>
#include <sstream>
#include <mutex>

#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/TCPServer.h>
#include <Poco/Net/TCPServerConnection.h>
#include <Poco/Net/TCPServerConnectionFactory.h>
#include <Poco/Thread.h>


using namespace drake;
using Eigen::VectorXd;

// Shared variables updated by network thread
std::array<double,7> g_tau{};                  // latest commanded torques
std::atomic<bool>   g_stop{false};             // true after "STOP" packet
std::mutex          g_tau_mutex;

// Poco TCP server connection handler
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
                if (n <= 0) break;  // connection closed
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
                    std::array<double,7> tau{};
                    if (!(iss >> t)) continue;  // malformed
                    for (int i=0;i<7 && iss; ++i) {
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

// Poco TCP Server Factory
class TorqueConnectionFactory : public Poco::Net::TCPServerConnectionFactory {
public:
    Poco::Net::TCPServerConnection* createConnection(const Poco::Net::StreamSocket& socket) override {
        return new TorqueConnectionHandler(socket);
    }
};


int main(int argc, char* argv[]) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0]
              << " <package.xml> <arm.urdf> <time_step>\n";
    return 1;
  }
  const std::string  package_xml = argv[1];
  const std::string  urdf        = argv[2];
  const double       dt          = std::stod(argv[3]);

  systems::DiagramBuilder<double> builder;

  // -------------------------------------------------------------
  // 1) Construct the MultibodyPlant + SceneGraph
  // -------------------------------------------------------------
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, dt);

  multibody::Parser parser(&plant, &scene_graph);
  parser.package_map().AddPackageXml(package_xml);
  const auto robot = parser.AddModels(urdf)[0];

  // Weld the "base" frame to the world.
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base", robot));

  // -------------------------------------------------------------
  // 2) Add joint actuators
  // -------------------------------------------------------------
  struct J { std::string n; double eff; };
  const std::vector<J> act = {
      {"fer_joint1", 87},
      {"fer_joint2", 87},
      {"fer_joint3", 87},
      {"fer_joint4", 87},
      {"fer_joint5", 12},
      {"fer_joint6", 87},
      {"fer_joint7", 87},
  };
  for (const auto& a : act) {
    const auto& joint =
        plant.GetJointByName<multibody::RevoluteJoint>(a.n, robot);
    plant.AddJointActuator(a.n + "_act", joint, a.eff);
  }

  // -------------------------------------------------------------
  // Add a tiny "triad" (RGB cylinders) to visualize every joint
  // frame's local axes in Meshcat.  We have to register the
  // geometries *before* MultibodyPlant::Finalize().
  // -------------------------------------------------------------
  const double axis_len = 0.15;   // [m]
  const double axis_rad = 0.003;  // [m]

  const geometry::Cylinder cyl_shape(axis_rad, axis_len);

  // Local transforms that place the cylinder so its base sits at the
  // frame origin and it extends +X / +Y / +Z respectively.
  const math::RigidTransformd X_FGx(
      math::RotationMatrixd::MakeYRotation(M_PI / 2),  // Z-axis → +X
      Eigen::Vector3d(axis_len / 2, 0, 0));
  const math::RigidTransformd X_FGy(
      math::RotationMatrixd::MakeXRotation(-M_PI / 2), // Z-axis → +Y
      Eigen::Vector3d(0, axis_len / 2, 0));
  const math::RigidTransformd X_FGz(
      math::RotationMatrixd(),               // already +Z
      Eigen::Vector3d(0, 0, axis_len / 2));

  const Eigen::Vector4d red(1, 0, 0, 0.9);
  const Eigen::Vector4d grn(0, 1, 0, 0.9);
  const Eigen::Vector4d blu(0, 0, 1, 0.9);

  for (const auto& a : act) {
    const auto& joint =
        plant.GetJointByName<multibody::RevoluteJoint>(a.n, robot);

    // We'll render the triad on the child body frame of this joint. It has
    // the same pose as the joint's frame-on-child and therefore reflects the
    // joint axis orientation in Meshcat.
    const auto& body = joint.child_body();

    plant.RegisterVisualGeometry(body, X_FGx, cyl_shape, a.n + ":x", red);
    plant.RegisterVisualGeometry(body, X_FGy, cyl_shape, a.n + ":y", grn);
    plant.RegisterVisualGeometry(body, X_FGz, cyl_shape, a.n + ":z", blu);
  }

  // -------------------------------------------------------------
  // 3) Set per-joint viscous damping to a nonzero value
  // -------------------------------------------------------------
  const std::vector<std::pair<std::string,double>> damp = {
      {"fer_joint1", 1.0},
      {"fer_joint2", 1.0},
      {"fer_joint3", 1.0},
      {"fer_joint4", 1.0},
      {"fer_joint5", 1.0},
      {"fer_joint6", 1.0},
      {"fer_joint7", 1.0},
  };
  for (const auto& [name, d] : damp) {
    plant.GetMutableJointByName<multibody::RevoluteJoint>(name, robot)
         .set_default_damping(d);
  }

  // -------------------------------------------------------------
  // Disable collisions by stripping proximity roles
  // -------------------------------------------------------------
  {
    const geometry::SourceId sid = plant.get_source_id().value();
    for (multibody::BodyIndex b(0); b < plant.num_bodies(); ++b) {
      const auto& body = plant.get_body(b);
      for (const geometry::GeometryId gid : plant.GetCollisionGeometriesForBody(body)) {
        scene_graph.RemoveRole(sid, gid, geometry::Role::kProximity);
      }
    }
  }

  plant.Finalize();

  // Start the TCP server in a separate thread
  Poco::Net::ServerSocket svs(12345);
  Poco::Net::TCPServer srv(new TorqueConnectionFactory(), svs);
  srv.start();
  std::cout << "TCP server started on port 12345" << std::endl;

  // -------------------------------------------------------------
  // 4) Build a "desired pose" VectorXd (q_des, v_des)
  // -------------------------------------------------------------
  const int n = plant.num_positions(robot);
  VectorXd q_des(n), v_des(plant.num_velocities(robot));
  q_des << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/2;
  v_des.setZero();  // all velocities = 0

  // -------------------------------------------------------------
  // 5) Add the InverseDynamics (GravityComp) block
  // -------------------------------------------------------------
  using systems::controllers::InverseDynamics;
  auto* g_comp = builder.AddSystem<InverseDynamics<double>>(
      &plant,
      InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);

  builder.Connect(plant.get_state_output_port(),
                g_comp->get_input_port_estimated_state());

  // -------------------------------------------------------------
  // 6) Add a System that provides torque from the network
  // -------------------------------------------------------------
  // Capture the Meshcat instance that AddDefaultVisualization will create.
  auto meshcat = std::make_shared<geometry::Meshcat>();
  visualization::AddDefaultVisualization(&builder, meshcat);

  // LeafSystem that outputs the latest 7-D torque vector
  class NetworkTorqueSystem final : public systems::LeafSystem<double> {
   public:
     NetworkTorqueSystem() {
       this->DeclareVectorOutputPort("tau_vec", systems::BasicVector<double>(7),
                                    &NetworkTorqueSystem::DoCalcOutput);
     }
   private:
     void DoCalcOutput(const systems::Context<double>&,
                       systems::BasicVector<double>* out) const {
       std::array<double,7> local;
       {
         std::lock_guard<std::mutex> lock(g_tau_mutex);
         local = g_tau;
       }
       Eigen::VectorXd v(7);
       for (int i=0;i<7;++i) v[i]=local[i];
       out->SetFromVector(v);
     }
  };

  auto* tau_from_network = builder.AddSystem<NetworkTorqueSystem>();

  // -------------------------------------------------------------
  // 7) Sum τ = τ_gc + τ_network
  // -------------------------------------------------------------
  auto* adder = builder.AddSystem<systems::Adder<double>>(2, n);

  builder.Connect(g_comp->get_output_port(), adder->get_input_port(0));
  builder.Connect(tau_from_network->get_output_port(), adder->get_input_port(1));

  // Feed the total torque to the plant.
  builder.Connect(adder->get_output_port(), plant.get_actuation_input_port());

  // -------------------------------------------------------------
  // 8) (Optional) Add Drake Visualizer & LogSinks
  // -------------------------------------------------------------
  // visualization::AddDefaultVisualization(&builder);

  const int n_state = plant.num_positions() + plant.num_velocities();
  auto* logger =
      builder.AddSystem<systems::VectorLogSink<double>>(n_state);
  builder.Connect(plant.get_state_output_port(), logger->get_input_port());

  auto* torque_logger =
      builder.AddSystem<systems::VectorLogSink<double>>(plant.num_actuators(robot));
  builder.Connect(adder->get_output_port(), torque_logger->get_input_port());

  // -------------------------------------------------------------
  // 9) Build the diagram + simulate
  // -------------------------------------------------------------
  auto diagram = builder.Build();
  systems::Simulator<double> sim(*diagram);
  sim.set_target_realtime_rate(1.0);

  auto& rk = sim.reset_integrator<systems::RungeKutta5Integrator<double>>();
  rk.set_target_accuracy(1e-12);
  rk.set_maximum_step_size(1e-4);

  // Set the initial joint state to exactly (q_des, v_des)
  auto& root = sim.get_mutable_context();
  auto& plant_ctx = plant.GetMyMutableContextFromRoot(&root);
  plant.SetPositions(&plant_ctx, robot, q_des);
  plant.SetVelocities(&plant_ctx, robot, v_des);

  sim.Initialize();
  std::cout << "Running … press <Enter> to begin"; std::cin.get();

  const double step = 0.02;  // 20 ms chunks of simulated time
  double target = 0.0;
  while (!g_stop.load() && target < 120.0) {
    target += step;
    sim.AdvanceTo(target);
  }

  srv.stop();

  // Dump a few logged samples
  const auto& L = logger->GetLog(logger->GetMyContextFromRoot(sim.get_context()));
  // Create a separate context for forward-kinematics queries (so we don't
  // disturb the simulator's final context when we tweak the positions for
  // every log sample).
  std::unique_ptr<systems::Context<double>> kinematics_ctx =
      plant.CreateDefaultContext();
  for (int i = 0; i < L.num_samples(); i += 200) {
    std::cout << L.sample_times()[i] << "  "
              << L.data().col(i).head(n).transpose() << "\n";
    for (int j = 0; j < 7; ++j) {
      std::cout << "  tau " << (j + 1) << " = "
                << torque_logger->GetLog(
                       torque_logger->GetMyContextFromRoot(sim.get_context()))
                       .data()(j, i)
                << "  [Nm]\n";
    }

    // -------------------------------------------------------------------
    // Forward-kinematics: report the world-space xyz position of every
    // joint frame.
    // -------------------------------------------------------------------
    // Set the joint configuration in our scratch context to match this log
    // sample's positions.
    const VectorXd q_sample = L.data().col(i).head(n);
    plant.SetPositions(kinematics_ctx.get(), robot, q_sample);

    for (const auto& a : act) {
      const auto& joint_obj =
          plant.GetJointByName<multibody::RevoluteJoint>(a.n, robot);

      // World pose of the joint's parent frame (this corresponds to the
      // joint origin as defined in the URDF).
      const math::RigidTransformd X_WJ = plant.CalcRelativeTransform(
          *kinematics_ctx, plant.world_frame(), joint_obj.frame_on_parent());

      const auto& p_WJ = X_WJ.translation();
      std::cout << "  xyz " << a.n << " = [" << p_WJ.transpose() << "]\n";
    }
  }

  return 0;
} 