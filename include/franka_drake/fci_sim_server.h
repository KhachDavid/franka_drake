#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <cstdint>
#include <string>
#include <functional>
#include <sys/socket.h>
#include <netinet/in.h>

#include "franka_drake/protocol/service_types.h"

namespace franka_fci_sim {

class FrankaFciSimServer {
 public:
  FrankaFciSimServer(uint16_t tcp_port = 1337, uint16_t udp_port = 1338);
  ~FrankaFciSimServer();

  // Start the server (blocking call)
  void run();

  // Stop the server
  void stop();

  // Set the callback to get the current RobotState from the simulation
  using StateProvider = std::function<protocol::RobotState()>;
  void set_state_provider(StateProvider provider);

  // Set the callback to handle received RobotCommand
  using CommandHandler = std::function<void(const protocol::RobotCommand&)>;
  void set_command_handler(CommandHandler handler);

 private:
  void server_thread();
  void handle_client_connection();
  void udp_control_loop();
  void close_sockets();

  uint16_t tcp_port_;
  uint16_t udp_port_;
  std::unique_ptr<std::thread> thread_;
  std::atomic<bool> running_;

  StateProvider state_provider_;
  CommandHandler command_handler_;

  // POSIX socket members
  int tcp_server_fd_ = -1;
  int tcp_client_fd_ = -1;
  int udp_socket_fd_ = -1;
  
  // UDP client address (set during TCP handshake)
  struct sockaddr_in udp_client_addr_;
  bool udp_client_ready_ = false;
  
  uint64_t message_id_ = 0;
};

}  // namespace franka_fci_sim 