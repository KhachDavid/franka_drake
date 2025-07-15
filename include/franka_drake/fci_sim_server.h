#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <cstdint>
#include <string>

#include "franka_drake/protocol/rbk_types.h"

namespace franka_fci_sim {

class FrankaFciSimServer {
 public:
  FrankaFciSimServer(uint16_t port = 30200);
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

  uint16_t port_;
  std::unique_ptr<std::thread> thread_;
  std::atomic<bool> running_;

  StateProvider state_provider_;
  CommandHandler command_handler_;

  // POSIX socket members (single client)
  int server_fd_ = -1;
  int client_fd_ = -1;
  void close_sockets();
  // TODO: Add handshake logic and Drake integration
};

}  // namespace franka_fci_sim 