#include "franka_drake/fci_sim_server.h"

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

namespace franka_fci_sim {

namespace {
// Helper to read exactly n bytes from a socket
bool read_exact(int fd, void* buf, size_t n) {
  size_t total = 0;
  char* ptr = static_cast<char*>(buf);
  while (total < n) {
    ssize_t r = recv(fd, ptr + total, n - total, 0);
    if (r <= 0) return false;
    total += r;
  }
  return true;
}
// Helper to write exactly n bytes to a socket
bool write_exact(int fd, const void* buf, size_t n) {
  size_t total = 0;
  const char* ptr = static_cast<const char*>(buf);
  while (total < n) {
    ssize_t w = send(fd, ptr + total, n - total, 0);
    if (w <= 0) return false;
    total += w;
  }
  return true;
}
} // namespace

FrankaFciSimServer::FrankaFciSimServer(uint16_t port)
    : port_(port), running_(false) {}

FrankaFciSimServer::~FrankaFciSimServer() {
  stop();
}

void FrankaFciSimServer::run() {
  running_ = true;
  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    std::cerr << "[FCI Sim Server] Failed to create socket" << std::endl;
    return;
  }

  int opt = 1;
  setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port_);

  if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    std::cerr << "[FCI Sim Server] Failed to bind socket" << std::endl;
    close_sockets();
    return;
  }

  if (listen(server_fd_, 1) < 0) {
    std::cerr << "[FCI Sim Server] Failed to listen" << std::endl;
    close_sockets();
    return;
  }

  std::cout << "[FCI Sim Server] Listening on port " << port_ << std::endl;

  sockaddr_in client_addr{};
  socklen_t client_len = sizeof(client_addr);
  client_fd_ = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
  if (client_fd_ < 0) {
    std::cerr << "[FCI Sim Server] Failed to accept client" << std::endl;
    close_sockets();
    return;
  }
  std::cout << "[FCI Sim Server] Client connected!" << std::endl;

  // --- FCI Handshake ---
  // 1. Connect
  protocol::Connect::Request connect_req{0};
  if (!read_exact(client_fd_, &connect_req, sizeof(connect_req))) {
    std::cerr << "[FCI Sim Server] Failed to read Connect::Request" << std::endl;
    close_sockets();
    return;
  }
  std::cout << "[FCI Sim Server] Received Connect::Request (udp_port=" << connect_req.udp_port << ")" << std::endl;
  protocol::Connect::Response connect_resp(protocol::Connect::Status::kSuccess);
  if (!write_exact(client_fd_, &connect_resp, sizeof(connect_resp))) {
    std::cerr << "[FCI Sim Server] Failed to send Connect::Response" << std::endl;
    close_sockets();
    return;
  }
  std::cout << "[FCI Sim Server] Sent Connect::Response" << std::endl;

  // 2. Move
  protocol::Move::Request move_req{protocol::Move::ControllerMode::kJointImpedance,
                                   protocol::Move::MotionGeneratorMode::kJointPosition,
                                   protocol::Move::Deviation(0,0,0),
                                   protocol::Move::Deviation(0,0,0)};
  if (!read_exact(client_fd_, &move_req, sizeof(move_req))) {
    std::cerr << "[FCI Sim Server] Failed to read Move::Request" << std::endl;
    close_sockets();
    return;
  }
  std::cout << "[FCI Sim Server] Received Move::Request" << std::endl;
  protocol::Move::Response move_resp(protocol::Move::Status::kSuccess);
  if (!write_exact(client_fd_, &move_resp, sizeof(move_resp))) {
    std::cerr << "[FCI Sim Server] Failed to send Move::Response" << std::endl;
    close_sockets();
    return;
  }
  std::cout << "[FCI Sim Server] Sent Move::Response" << std::endl;

  // --- Main control loop ---
  while (running_) {
    protocol::RobotCommand cmd{};
    if (!read_exact(client_fd_, &cmd, sizeof(cmd))) {
      std::cerr << "[FCI Sim Server] Client disconnected or error (recv RobotCommand)" << std::endl;
      break;
    }
    if (command_handler_) {
      command_handler_(cmd);
    }
    protocol::RobotState state{};
    if (state_provider_) {
      state = state_provider_();
    }
    if (!write_exact(client_fd_, &state, sizeof(state))) {
      std::cerr << "[FCI Sim Server] Client disconnected or error (send RobotState)" << std::endl;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  close_sockets();
  std::cout << "[FCI Sim Server] Server stopped." << std::endl;
}

void FrankaFciSimServer::stop() {
  running_ = false;
  close_sockets();
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
}

void FrankaFciSimServer::set_state_provider(StateProvider provider) {
  state_provider_ = std::move(provider);
}

void FrankaFciSimServer::set_command_handler(CommandHandler handler) {
  command_handler_ = std::move(handler);
}

void FrankaFciSimServer::close_sockets() {
  if (client_fd_ >= 0) {
    close(client_fd_);
    client_fd_ = -1;
  }
  if (server_fd_ >= 0) {
    close(server_fd_);
    server_fd_ = -1;
  }
}

}  // namespace franka_fci_sim 