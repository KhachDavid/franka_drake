#include "franka_drake/gripper_sim_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

namespace franka_fci_sim {

using research_interface::gripper::CommandHeader;
using research_interface::gripper::Connect;
using research_interface::gripper::GripperState;
using research_interface::gripper::Homing;
using research_interface::gripper::Move;
using research_interface::gripper::Stop;

namespace {
bool read_exact(int fd, void* buf, size_t n) {
  size_t total = 0; char* p = static_cast<char*>(buf);
  while (total < n) { ssize_t r = recv(fd, p + total, n - total, 0); if (r <= 0) return false; total += r; }
  return true;
}
bool write_exact(int fd, const void* buf, size_t n) {
  size_t total = 0; const char* p = static_cast<const char*>(buf);
  while (total < n) { ssize_t w = send(fd, p + total, n - total, 0); if (w <= 0) return false; total += w; }
  return true;
}
}

FrankaGripperSimServer::FrankaGripperSimServer(uint16_t port,
                                               GetWidthFn get_width,
                                               SetMoveFn set_move,
                                               HomingFn homing)
    : get_width_(std::move(get_width)), set_move_(std::move(set_move)), homing_(std::move(homing)), port_(port) {}

FrankaGripperSimServer::~FrankaGripperSimServer() { stop(); }

void FrankaGripperSimServer::run() {
  running_ = true;
  tcp_server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (tcp_server_fd_ < 0 || udp_socket_fd_ < 0) { std::cerr << "[GripperSim] socket() failed" << std::endl; return; }
  int opt = 1; setsockopt(tcp_server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)); setsockopt(udp_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  sockaddr_in tcp_addr{}; tcp_addr.sin_family = AF_INET; tcp_addr.sin_addr.s_addr = INADDR_ANY; tcp_addr.sin_port = htons(port_);
  if (bind(tcp_server_fd_, (sockaddr*)&tcp_addr, sizeof(tcp_addr)) < 0) { std::cerr << "[GripperSim] bind TCP failed" << std::endl; close_sockets(); return; }
  sockaddr_in udp_addr{}; udp_addr.sin_family = AF_INET; udp_addr.sin_addr.s_addr = INADDR_ANY; udp_addr.sin_port = htons(port_);
  if (bind(udp_socket_fd_, (sockaddr*)&udp_addr, sizeof(udp_addr)) < 0) { std::cerr << "[GripperSim] bind UDP failed" << std::endl; close_sockets(); return; }
  if (listen(tcp_server_fd_, 1) < 0) { std::cerr << "[GripperSim] listen failed" << std::endl; close_sockets(); return; }

  while (running_) {
    sockaddr_in client_addr{}; socklen_t len = sizeof(client_addr);
    tcp_client_fd_ = accept(tcp_server_fd_, (sockaddr*)&client_addr, &len);
    if (tcp_client_fd_ < 0) continue;
    udp_client_addr_ = client_addr; udp_client_ready_ = false;
    handle_client_connection();
    if (tcp_client_fd_ >= 0) { close(tcp_client_fd_); tcp_client_fd_ = -1; }
  }
  close_sockets();
}

void FrankaGripperSimServer::handle_client_connection() {
  // Handle Connect
  CommandHeader header; if (!read_exact(tcp_client_fd_, &header, sizeof(header))) return;
  // Header is already the correct type; size check only
  if (header.size < sizeof(header)) return;
  handle_connect_command(header);
  // TCP loop
  udp_client_ready_ = true;
  std::thread([this]{ udp_state_loop(); }).detach();
  handle_tcp_commands();
  udp_client_ready_ = false;
}

void FrankaGripperSimServer::handle_connect_command(const CommandHeader& header) {
  // Read request
  typename Connect::Request req(uint16_t{0}); // placeholder; actual struct read below
  if (header.size > sizeof(header)) {
    std::vector<uint8_t> payload(header.size - sizeof(header));
    if (read_exact(tcp_client_fd_, payload.data(), payload.size())) {
      const auto* r = reinterpret_cast<const Connect::Request*>(payload.data());
      // Update UDP destination port from client's requested port
      udp_client_addr_.sin_port = htons(r->udp_port);
    }
  }
  // Send response
  typename Connect::Response resp(Connect::Status::kSuccess);
  CommandHeader resp_h(research_interface::gripper::Command::kConnect, header.command_id,
                       static_cast<uint32_t>(sizeof(CommandHeader) + sizeof(resp)));
  write_exact(tcp_client_fd_, &resp_h, sizeof(resp_h));
  write_exact(tcp_client_fd_, &resp, sizeof(resp));
}

void FrankaGripperSimServer::handle_tcp_commands() {
  while (running_ && udp_client_ready_) {
    CommandHeader header; if (!read_exact(tcp_client_fd_, &header, sizeof(header))) return;
    switch (static_cast<research_interface::gripper::Command>(header.command)) {
      case research_interface::gripper::Command::kHoming: {
        if (homing_) homing_();
        typename Homing::Response resp(Homing::Status::kSuccess);
        CommandHeader h(research_interface::gripper::Command::kHoming, header.command_id,
                        static_cast<uint32_t>(sizeof(CommandHeader) + sizeof(resp)));
        write_exact(tcp_client_fd_, &h, sizeof(h)); write_exact(tcp_client_fd_, &resp, sizeof(resp));
        break;
      }
      case research_interface::gripper::Command::kMove: {
        // Read Move::Request (has const members; avoid assignment)
        double width = 0.0, speed = 0.0;
        if (header.size > sizeof(header)) {
          std::vector<uint8_t> payload(header.size - sizeof(header));
          read_exact(tcp_client_fd_, payload.data(), payload.size());
          const auto* reqp = reinterpret_cast<const Move::Request*>(payload.data());
          width = reqp->width;
          speed = reqp->speed;
        }
        if (set_move_) set_move_(width, speed);
        typename Move::Response resp(Move::Status::kSuccess);
        CommandHeader h(research_interface::gripper::Command::kMove, header.command_id,
                        static_cast<uint32_t>(sizeof(CommandHeader) + sizeof(resp)));
        write_exact(tcp_client_fd_, &h, sizeof(h)); write_exact(tcp_client_fd_, &resp, sizeof(resp));
        break;
      }
      case research_interface::gripper::Command::kStop: {
        typename Stop::Response resp(Stop::Status::kSuccess);
        CommandHeader h(research_interface::gripper::Command::kStop, header.command_id,
                        static_cast<uint32_t>(sizeof(CommandHeader) + sizeof(resp)));
        write_exact(tcp_client_fd_, &h, sizeof(h)); write_exact(tcp_client_fd_, &resp, sizeof(resp));
        break;
      }
      case research_interface::gripper::Command::kGrasp: {
        // Accept and return success for now
        std::vector<uint8_t> payload(header.size - sizeof(header));
        if (!payload.empty()) read_exact(tcp_client_fd_, payload.data(), payload.size());
        research_interface::gripper::Grasp::Response resp(research_interface::gripper::Grasp::Status::kSuccess);
        CommandHeader h(research_interface::gripper::Command::kGrasp, header.command_id,
                        static_cast<uint32_t>(sizeof(CommandHeader) + sizeof(resp)));
        write_exact(tcp_client_fd_, &h, sizeof(h)); write_exact(tcp_client_fd_, &resp, sizeof(resp));
        break;
      }
      default: {
        // Consume payload if any, respond success
        std::vector<uint8_t> payload(header.size - sizeof(header));
        if (!payload.empty()) read_exact(tcp_client_fd_, payload.data(), payload.size());
        uint8_t ok = 0; CommandHeader h(static_cast<research_interface::gripper::Command>(header.command), header.command_id,
                                        static_cast<uint32_t>(sizeof(CommandHeader) + 1));
        write_exact(tcp_client_fd_, &h, sizeof(h)); write_exact(tcp_client_fd_, &ok, sizeof(ok));
        break;
      }
    }
  }
}

void FrankaGripperSimServer::udp_state_loop() {
  udp_thread_running_ = true;
  while (running_ && udp_client_ready_) {
    GripperState st{};
    st.message_id = ++message_id_;
    st.width = get_width_ ? get_width_() : 0.0;
    st.max_width = 0.08;
    st.is_grasped = false;
    st.temperature = 0.0;
    // No time field in GripperState here
    sendto(udp_socket_fd_, &st, sizeof(st), 0, (sockaddr*)&udp_client_addr_, sizeof(udp_client_addr_));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  udp_thread_running_ = false;
}

void FrankaGripperSimServer::stop() {
  running_ = false; udp_client_ready_ = false; close_sockets();
  if (thread_ && thread_->joinable()) thread_->join();
}

void FrankaGripperSimServer::close_sockets() {
  if (tcp_client_fd_ >= 0) { close(tcp_client_fd_); tcp_client_fd_ = -1; }
  if (tcp_server_fd_ >= 0) { close(tcp_server_fd_); tcp_server_fd_ = -1; }
  if (udp_socket_fd_ >= 0) { close(udp_socket_fd_); udp_socket_fd_ = -1; }
}

}  // namespace franka_fci_sim


