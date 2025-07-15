#include "franka_drake/fci_sim_server.h"
#include "franka_drake/protocol/service_types.h"

#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>  // For inet_ntoa
#include <unistd.h>
#include <chrono>
#include <thread>

// Add logging function
void log_message(const std::string& msg) {
  std::ofstream log("/tmp/fci_server.log", std::ios::app);
  log << msg << std::endl;
  std::cout << msg << std::endl;
}

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

FrankaFciSimServer::FrankaFciSimServer(uint16_t tcp_port, uint16_t udp_port)
    : tcp_port_(tcp_port), udp_port_(udp_port), running_(false) {}

FrankaFciSimServer::~FrankaFciSimServer() {
  stop();
}

void FrankaFciSimServer::run() {
  running_ = true;
  
  // Create TCP socket for handshake
  tcp_server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_server_fd_ < 0) {
    std::cerr << "[FCI Sim Server] Failed to create TCP socket" << std::endl;
    return;
  }

  // Create UDP socket for control loop
  udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket_fd_ < 0) {
    std::cerr << "[FCI Sim Server] Failed to create UDP socket" << std::endl;
    close_sockets();
    return;
  }

  int opt = 1;
  setsockopt(tcp_server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  setsockopt(udp_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  // Bind TCP socket
  sockaddr_in tcp_addr{};
  tcp_addr.sin_family = AF_INET;
  tcp_addr.sin_addr.s_addr = INADDR_ANY;
  tcp_addr.sin_port = htons(tcp_port_);

  std::cout << "[FCI Sim Server] Binding TCP to port " << tcp_port_ << std::endl;

  if (bind(tcp_server_fd_, (struct sockaddr*)&tcp_addr, sizeof(tcp_addr)) < 0) {
    std::cerr << "[FCI Sim Server] Failed to bind TCP socket" << std::endl;
    close_sockets();
    return;
  }

  // Bind UDP socket
  sockaddr_in udp_addr{};
  udp_addr.sin_family = AF_INET;
  udp_addr.sin_addr.s_addr = INADDR_ANY;
  udp_addr.sin_port = htons(udp_port_);

  std::cout << "[FCI Sim Server] Binding UDP to port " << udp_port_ << std::endl;

  if (bind(udp_socket_fd_, (struct sockaddr*)&udp_addr, sizeof(udp_addr)) < 0) {
    std::cerr << "[FCI Sim Server] Failed to bind UDP socket" << std::endl;
    close_sockets();
    return;
  }

  if (listen(tcp_server_fd_, 1) < 0) {
    std::cerr << "[FCI Sim Server] Failed to listen on TCP" << std::endl;
    close_sockets();
    return;
  }
  
  std::cout << "[FCI Sim Server] Listening on TCP port " << tcp_port_ << " and UDP port " << udp_port_ << std::endl;

  while (running_) {
    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    tcp_client_fd_ = accept(tcp_server_fd_, (struct sockaddr*)&client_addr, &client_len);
    if (tcp_client_fd_ < 0) {
      if (running_) {
        std::cerr << "[FCI Sim Server] Failed to accept client" << std::endl;
      }
      break;
    }
    log_message("[FCI Sim Server] Client connected!");

    // Store client address for UDP communication
    udp_client_addr_ = client_addr;
    udp_client_ready_ = false;

    // Handle the client connection
    handle_client_connection();
    
    // Close client socket and continue listening
    if (tcp_client_fd_ >= 0) {
      close(tcp_client_fd_);
      tcp_client_fd_ = -1;
    }
    udp_client_ready_ = false;
    std::cout << "[FCI Sim Server] Client disconnected, ready for next connection" << std::endl;
  }

  close_sockets();
  std::cout << "[FCI Sim Server] Server stopped." << std::endl;
}

void FrankaFciSimServer::handle_client_connection() {
  // --- FCI Handshake via TCP ---
  // 1. Connect - First read the command header to understand the message structure
  log_message("[FCI Sim Server] Reading TCP command header...");
  
  struct CommandHeader {
    uint32_t command;
    uint32_t command_id;
    uint32_t size;
  };
  
  CommandHeader header;
  if (!read_exact(tcp_client_fd_, &header, sizeof(header))) {
    log_message("[FCI Sim Server] Failed to read command header");
    return;
  }
  
  char header_msg[256];
  sprintf(header_msg, "[FCI Sim Server] Command header: command=%u, command_id=%u, size=%u", 
          header.command, header.command_id, header.size);
  log_message(header_msg);
  
  // Read the actual payload
  if (header.size > sizeof(header)) {
    size_t payload_size = header.size - sizeof(header);
    std::vector<uint8_t> payload(payload_size);
    
    if (!read_exact(tcp_client_fd_, payload.data(), payload_size)) {
      log_message("[FCI Sim Server] Failed to read payload");
      return;
    }
    
    char payload_msg[512];
    sprintf(payload_msg, "[FCI Sim Server] Payload (%lu bytes): ", payload_size);
    std::string payload_hex = payload_msg;
    for (size_t i = 0; i < std::min(payload_size, (size_t)32); ++i) {
      char hex_str[4];
      sprintf(hex_str, "%02x ", payload[i]);
      payload_hex += hex_str;
    }
    log_message(payload_hex);
    
    // Try to parse as Connect::Request if payload size matches
    if (payload_size >= sizeof(protocol::Connect::Request)) {
      const protocol::Connect::Request* connect_req = 
        reinterpret_cast<const protocol::Connect::Request*>(payload.data());
      
      char connect_msg[256];
      sprintf(connect_msg, "[FCI Sim Server] Parsed Connect::Request: version=%d, udp_port=%d", 
              connect_req->version, connect_req->udp_port);
      log_message(connect_msg);
      
      // Update UDP client address with the port from libfranka
      udp_client_addr_.sin_port = htons(connect_req->udp_port);
    }
  }
  
  protocol::Connect::Response connect_resp(protocol::Connect::Status::kSuccess);
  if (!write_exact(tcp_client_fd_, &connect_resp, sizeof(connect_resp))) {
    log_message("[FCI Sim Server] Failed to send Connect::Response");
    return;
  }
  log_message("[FCI Sim Server] Sent Connect::Response");

  // 2. Move - libfranka should send this after Connect
  log_message("[FCI Sim Server] Expecting Move::Request...");
  
  CommandHeader move_header;
  if (!read_exact(tcp_client_fd_, &move_header, sizeof(move_header))) {
    log_message("[FCI Sim Server] Failed to read Move command header");
    return;
  }
  
  char header_debug[256];
  sprintf(header_debug, "[FCI Sim Server] Move header: command=%u, command_id=%u, size=%u", 
          move_header.command, move_header.command_id, move_header.size);
  log_message(header_debug);
  
  // Read Move payload if present (should contain ControllerMode, MotionGeneratorMode, etc.)
  if (move_header.size > sizeof(move_header)) {
    size_t payload_size = move_header.size - sizeof(move_header);
    std::vector<uint8_t> move_payload(payload_size);
    
    if (!read_exact(tcp_client_fd_, move_payload.data(), payload_size)) {
      char error_msg[256];
      sprintf(error_msg, "[FCI Sim Server] Failed to read Move payload (%lu bytes)", payload_size);
      log_message(error_msg);
      return;
    }
    
    char payload_debug[256];
    sprintf(payload_debug, "[FCI Sim Server] Move payload (%lu bytes): ", payload_size);
    std::string payload_hex = payload_debug;
    for (size_t i = 0; i < std::min(payload_size, (size_t)8); ++i) {
      char hex_str[4];
      sprintf(hex_str, "%02x ", move_payload[i]);
      payload_hex += hex_str;
    }
    log_message(payload_hex);
  }
  
  // Send Move::Response indicating motion can start
  protocol::Move::Response move_resp(protocol::Move::Status::kMotionStarted);
  if (!write_exact(tcp_client_fd_, &move_resp, sizeof(move_resp))) {
    log_message("[FCI Sim Server] Failed to send Move::Response");
    return;
  }
  log_message("[FCI Sim Server] Sent Move::Response (motion started)");

  // NOW start UDP communication after proper Move handshake
  log_message("[FCI Sim Server] Move handshake complete - starting UDP control loop...");
  udp_client_ready_ = true;
  message_id_ = 1;
  
  udp_control_loop();
}

void FrankaFciSimServer::udp_control_loop() {
  log_message("[FCI Sim Server] UDP control loop started");
  
  // Debug: Print UDP client address
  char addr_debug[256];
  sprintf(addr_debug, "[FCI Sim Server] UDP client address: %s:%d", 
          inet_ntoa(udp_client_addr_.sin_addr), ntohs(udp_client_addr_.sin_port));
  log_message(addr_debug);
  
  // Send initial state
  if (state_provider_) {
    protocol::RobotState state = state_provider_();
    state.message_id = message_id_;
    state.robot_mode = protocol::RobotMode::kMove;
    state.motion_generator_mode = protocol::MotionGeneratorMode::kCartesianPosition;
    state.controller_mode = protocol::ControllerMode::kJointImpedance;
    state.control_command_success_rate = 1.0;
    
    ssize_t sent = sendto(udp_socket_fd_, &state, sizeof(state), 0,
                         (struct sockaddr*)&udp_client_addr_, sizeof(udp_client_addr_));
    if (sent != sizeof(state)) {
      char error_msg[256];
      sprintf(error_msg, "[FCI Sim Server] Failed to send initial UDP state: sent=%ld, expected=%lu", sent, sizeof(state));
      log_message(error_msg);
      return;
    }
    log_message("[FCI Sim Server] Sent initial RobotState via UDP");
  }
  
  int loop_count = 0;
  
  // Control loop
  while (running_ && udp_client_ready_) {
    // Wait for RobotCommand via UDP
    protocol::RobotCommand command{};
    sockaddr_in from_addr{};
    socklen_t from_len = sizeof(from_addr);
    
    // Set timeout for UDP receive
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000; // 10ms timeout
    setsockopt(udp_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    ssize_t received = recvfrom(udp_socket_fd_, &command, sizeof(command), 0,
                               (struct sockaddr*)&from_addr, &from_len);
    
    // Debug UDP receive attempts every 1000 loops (every ~10 seconds)
    loop_count++;
    if (loop_count % 1000 == 0) {
      char debug_msg[256];
      sprintf(debug_msg, "[FCI Sim Server] UDP loop #%d: received=%ld, expected=%lu, errno=%d", 
              loop_count, received, sizeof(command), errno);
      log_message(debug_msg);
    }
    
    if (received == sizeof(command)) {
      // SUCCESS: Process the command
      char success_msg[256];
      sprintf(success_msg, "[FCI Sim Server] *** RECEIVED UDP COMMAND *** from %s:%d, msg_id=%lu", 
              inet_ntoa(from_addr.sin_addr), ntohs(from_addr.sin_port), command.message_id);
      log_message(success_msg);
      
      if (command_handler_) {
        command_handler_(command);
      }
      
      // Send updated state back
      if (state_provider_) {
        protocol::RobotState state = state_provider_();
        state.message_id = command.message_id + 1;
        state.robot_mode = protocol::RobotMode::kMove;
        state.motion_generator_mode = protocol::MotionGeneratorMode::kCartesianPosition;
        state.controller_mode = protocol::ControllerMode::kJointImpedance;
        state.control_command_success_rate = 1.0;
        
        ssize_t sent = sendto(udp_socket_fd_, &state, sizeof(state), 0,
                             (struct sockaddr*)&from_addr, sizeof(from_addr));
        
        char msg[256];
        sprintf(msg, "[FCI Sim Server] UDP exchange: received cmd %lu, sent state %lu", 
                command.message_id, state.message_id);
        log_message(msg);
        
        message_id_ = state.message_id;
      }
    } else if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      // Timeout - continue loop (this is normal)
      continue;
    } else {
      // Error or unexpected size - log occasionally
      if (received > 0 && loop_count % 500 == 0) {
        char msg[256];
        sprintf(msg, "[FCI Sim Server] Unexpected UDP packet size: %ld (expected %lu) from %s:%d", 
                received, sizeof(command), inet_ntoa(from_addr.sin_addr), ntohs(from_addr.sin_port));
        log_message(msg);
      }
      // Continue anyway
    }
    
    // 1kHz control loop timing
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }
  
  log_message("[FCI Sim Server] UDP control loop ended");
}

void FrankaFciSimServer::stop() {
  running_ = false;
  udp_client_ready_ = false;
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
  if (tcp_client_fd_ >= 0) {
    close(tcp_client_fd_);
    tcp_client_fd_ = -1;
  }
  if (tcp_server_fd_ >= 0) {
    close(tcp_server_fd_);
    tcp_server_fd_ = -1;
  }
  if (udp_socket_fd_ >= 0) {
    close(udp_socket_fd_);
    udp_socket_fd_ = -1;
  }
}

}  // namespace franka_fci_sim 