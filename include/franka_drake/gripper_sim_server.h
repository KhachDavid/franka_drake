#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <netinet/in.h>

#include <research_interface/gripper/types.h>

namespace franka_fci_sim {

// Minimal libfranka-compatible gripper server.
// Implements TCP command handling (Connect/Homing/Move/Stop/Grasp as no-op success)
// and UDP streaming of research_interface::gripper::GripperState.
class FrankaGripperSimServer {
 public:
  using GetWidthFn = std::function<double()>;                 // meters
  using SetMoveFn = std::function<void(double width, double speed)>;  // meters, m/s
  using HomingFn = std::function<void()>;

  FrankaGripperSimServer(uint16_t port,
                         GetWidthFn get_width,
                         SetMoveFn set_move,
                         HomingFn homing);
  ~FrankaGripperSimServer();

  void run();
  void stop();

 private:
  void close_sockets();
  void handle_client_connection();
  void handle_connect_command(const research_interface::gripper::CommandHeader& header);
  void handle_tcp_commands();
  void udp_state_loop();

  // networking
  int tcp_server_fd_{-1};
  int tcp_client_fd_{-1};
  int udp_socket_fd_{-1};
  sockaddr_in udp_client_addr_{};
  std::atomic<bool> running_{false};
  std::atomic<bool> udp_client_ready_{false};
  std::atomic<bool> udp_thread_running_{false};
  std::unique_ptr<std::thread> thread_{};

  // callbacks
  GetWidthFn get_width_;
  SetMoveFn set_move_;
  HomingFn homing_;

  // state
  std::atomic<uint64_t> message_id_{0};
  uint16_t port_;
};

}  // namespace franka_fci_sim


