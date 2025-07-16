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
    std::cerr << "[FCI Sim Server] Failed to bind TCP socket: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
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
    std::cerr << "[FCI Sim Server] Failed to bind UDP socket: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
    close_sockets();
    return;
  }

  if (listen(tcp_server_fd_, 1) < 0) {
    std::cerr << "[FCI Sim Server] Failed to listen on TCP: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
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
  
  protocol::CommandHeader header;
  if (!read_exact(tcp_client_fd_, &header, sizeof(header))) {
    log_message("[FCI Sim Server] Failed to read command header");
    return;
  }
  
  char header_msg[256];
  sprintf(header_msg, "[FCI Sim Server] Command header: command=%u, command_id=%u, size=%u", 
          static_cast<uint32_t>(header.command), header.command_id, header.size);
  log_message(header_msg);
  
  // Handle Connect command for initial handshake
  if (header.command == protocol::Command::kConnect) {
    handle_connect_command(header);
  } else {
    log_message("[FCI Sim Server] Expected Connect command, got something else during handshake");
    return;
  }
  
  // After successful Connect, handle other TCP commands
  handle_tcp_commands();
}

void FrankaFciSimServer::handle_connect_command(const protocol::CommandHeader& header) {
  // Read the actual payload
  if (header.size > sizeof(header)) {
    size_t payload_size = header.size - sizeof(header);
    std::vector<uint8_t> payload(payload_size);
    
    if (!read_exact(tcp_client_fd_, payload.data(), payload_size)) {
      log_message("[FCI Sim Server] Failed to read Connect payload");
      return;
    }
    
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
  
  // Send properly formatted Connect::Response with header
  protocol::CommandHeader response_header(
    protocol::Command::kConnect, 
    header.command_id, 
    sizeof(protocol::CommandHeader) + sizeof(protocol::Connect::Response)
  );
  
  protocol::Connect::Response connect_resp(protocol::Connect::Status::kSuccess);
  
  // Send header first
  if (!write_exact(tcp_client_fd_, &response_header, sizeof(response_header))) {
    log_message("[FCI Sim Server] Failed to send Connect response header");
    return;
  }
  
  // Send response payload
  if (!write_exact(tcp_client_fd_, &connect_resp, sizeof(connect_resp))) {
    log_message("[FCI Sim Server] Failed to send Connect::Response");
    return;
  }
  log_message("[FCI Sim Server] Sent Connect::Response with proper header");

  // CRITICAL: Send initial robot state via UDP IMMEDIATELY after Connect response
  // libfranka calls udpBlockingReceive right after Connect handshake!
  log_message("[FCI Sim Server] Sending initial robot state via UDP...");
  udp_client_ready_ = true;
  message_id_ = 1;
  
  // Send initial state immediately
  if (state_provider_) {
    protocol::RobotState initial_state = state_provider_();
    initial_state.message_id = message_id_++;
    initial_state.robot_mode = protocol::RobotMode::kIdle;
    initial_state.motion_generator_mode = protocol::MotionGeneratorMode::kIdle;
    initial_state.controller_mode = protocol::ControllerMode::kOther;
    initial_state.control_command_success_rate = 1.0;
    initial_state.errors.fill(false);
    initial_state.reflex_reason.fill(false);
    
    ssize_t sent = sendto(udp_socket_fd_, &initial_state, sizeof(initial_state), 0,
                         (struct sockaddr*)&udp_client_addr_, sizeof(udp_client_addr_));
    
    char msg[256];
    sprintf(msg, "[FCI Sim Server] Sent initial UDP state: %ld bytes, msg_id=%lu", 
            sent, initial_state.message_id);
    log_message(msg);
  }
  
  // Start continuous UDP state transmission for robot.read() support
  // This allows echo_robot_state and similar read-only clients to work
  log_message("[FCI Sim Server] Starting continuous UDP state transmission for robot.read() support");
  std::thread udp_state_thread([this]() {
    int loop_count = 0;
    auto last_state_time = std::chrono::steady_clock::now();
    
    while (running_ && udp_client_ready_) {
      auto current_time = std::chrono::steady_clock::now();
      
      // Send robot state at ~1kHz
      if (std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_state_time).count() >= 1000) {
        if (state_provider_) {
          protocol::RobotState state = state_provider_();
          state.message_id = message_id_++;
          
          // Set appropriate robot mode based on control state
          if (control_mode_active_.load()) {
            state.robot_mode = protocol::RobotMode::kMove;
            
            // Set appropriate modes based on requested motion type
            switch (requested_controller_mode_) {
              case protocol::Move::ControllerMode::kJointImpedance:
                state.controller_mode = protocol::ControllerMode::kJointImpedance;
                break;
              case protocol::Move::ControllerMode::kCartesianImpedance:
                state.controller_mode = protocol::ControllerMode::kCartesianImpedance;
                break;
              case protocol::Move::ControllerMode::kExternalController:
                state.controller_mode = protocol::ControllerMode::kExternalController;
                break;
              default:
                state.controller_mode = protocol::ControllerMode::kExternalController;
                break;
            }
            
            // FIXED: Always respect the requested motion generator mode from libfranka
            // The controller mode handles torque vs impedance, motion generator mode is separate
            switch (requested_motion_generator_mode_) {
              case protocol::Move::MotionGeneratorMode::kJointPosition:
                state.motion_generator_mode = protocol::MotionGeneratorMode::kJointPosition;
                break;
              case protocol::Move::MotionGeneratorMode::kJointVelocity:
                state.motion_generator_mode = protocol::MotionGeneratorMode::kJointVelocity;
                break;
              case protocol::Move::MotionGeneratorMode::kCartesianPosition:
                state.motion_generator_mode = protocol::MotionGeneratorMode::kCartesianPosition;
                break;
              case protocol::Move::MotionGeneratorMode::kCartesianVelocity:
                state.motion_generator_mode = protocol::MotionGeneratorMode::kCartesianVelocity;
                break;
              default:
                state.motion_generator_mode = protocol::MotionGeneratorMode::kJointPosition;
                break;
            }
          } else {
            // Read-only mode (for echo_robot_state)
            state.robot_mode = protocol::RobotMode::kIdle;
            state.motion_generator_mode = protocol::MotionGeneratorMode::kIdle;
            state.controller_mode = protocol::ControllerMode::kOther;
          }
          
          state.control_command_success_rate = 1.0;
          state.errors.fill(false);
          state.reflex_reason.fill(false);
          
          ssize_t sent = sendto(udp_socket_fd_, &state, sizeof(state), 0,
                               (struct sockaddr*)&udp_client_addr_, sizeof(udp_client_addr_));
          
          last_state_time = current_time;
          
          // Debug every 1000 loops (every ~1 second)
          if (loop_count % 1000 == 0) {
            char state_msg[256];
            sprintf(state_msg, "[FCI Sim Server] Sent robot state #%d: msg_id=%lu, sent=%ld bytes (mode: %s)", 
                    loop_count, state.message_id, sent, 
                    control_mode_active_.load() ? "CONTROL" : "READ_ONLY");
            log_message(state_msg);
          }
        }
      }
      
      // If in control mode, check for incoming UDP commands
      if (control_mode_active_.load()) {
        protocol::RobotCommand command{};
        sockaddr_in from_addr{};
        socklen_t from_len = sizeof(from_addr);
        
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100; // 100μs timeout (non-blocking)
        setsockopt(udp_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        
        ssize_t received = recvfrom(udp_socket_fd_, &command, sizeof(command), 0,
                                   (struct sockaddr*)&from_addr, &from_len);
        
        if (received == sizeof(command)) {
          // SUCCESS: Process the command
          char success_msg[256];
          sprintf(success_msg, "[FCI Sim Server] *** RECEIVED UDP COMMAND *** from %s:%d, msg_id=%lu", 
                  inet_ntoa(from_addr.sin_addr), ntohs(from_addr.sin_port), command.message_id);
          log_message(success_msg);
          
          if (command_handler_) {
            command_handler_(command);
          }
          
          // Send immediate response state
          if (state_provider_) {
            protocol::RobotState response_state = state_provider_();
            response_state.message_id = command.message_id + 1;
            response_state.robot_mode = protocol::RobotMode::kMove;
            
            // FIXED: For torque control (ExternalController), motion generator should be Idle
            response_state.controller_mode = protocol::ControllerMode::kExternalController;
            
            // Convert from Move::MotionGeneratorMode to MotionGeneratorMode
            switch (requested_motion_generator_mode_) {
              case protocol::Move::MotionGeneratorMode::kJointPosition:
                response_state.motion_generator_mode = protocol::MotionGeneratorMode::kJointPosition;
                break;
              case protocol::Move::MotionGeneratorMode::kJointVelocity:
                response_state.motion_generator_mode = protocol::MotionGeneratorMode::kJointVelocity;
                break;
              case protocol::Move::MotionGeneratorMode::kCartesianPosition:
                response_state.motion_generator_mode = protocol::MotionGeneratorMode::kCartesianPosition;
                break;
              case protocol::Move::MotionGeneratorMode::kCartesianVelocity:
                response_state.motion_generator_mode = protocol::MotionGeneratorMode::kCartesianVelocity;
                break;
              default:
                response_state.motion_generator_mode = protocol::MotionGeneratorMode::kJointPosition;
                break;
            }
            
            response_state.control_command_success_rate = 1.0;
            response_state.errors.fill(false);
            response_state.reflex_reason.fill(false);
            
            ssize_t sent = sendto(udp_socket_fd_, &response_state, sizeof(response_state), 0,
                                 (struct sockaddr*)&from_addr, sizeof(from_addr));
            
            char response_msg[256];
            sprintf(response_msg, "[FCI Sim Server] Sent UDP response: cmd_id=%lu -> state_id=%lu", 
                    command.message_id, response_state.message_id);
            log_message(response_msg);
            
            message_id_ = response_state.message_id;
          }
        }
      }
      
      loop_count++;
      std::this_thread::sleep_for(std::chrono::microseconds(1000)); // 1kHz
    }
    log_message("[FCI Sim Server] Background UDP state transmission stopped");
  });
  udp_state_thread.detach(); // Run in background
  
  // Small delay to ensure libfranka receives the initial state
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void FrankaFciSimServer::handle_tcp_commands() {
  log_message("[FCI Sim Server] Starting TCP command handling loop...");
  
  while (running_ && udp_client_ready_) {
    // Read command header with longer timeout to handle user interaction delays
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(tcp_client_fd_, &read_fds);
    struct timeval timeout = {30, 0}; // 30 second timeout to allow for user interaction
    
    int select_result = select(tcp_client_fd_ + 1, &read_fds, NULL, NULL, &timeout);
    
    if (select_result <= 0) {
      // Timeout or error - continue with background UDP state transmission
      if (select_result == 0) {
        log_message("[FCI Sim Server] No TCP commands received for 30s, continuing background state transmission");
        continue; // Keep handling TCP commands
      } else {
        log_message("[FCI Sim Server] TCP select error, closing connection");
        return;
      }
    }
    
    // Read command header
    protocol::CommandHeader cmd_header;
    if (!read_exact(tcp_client_fd_, &cmd_header, sizeof(cmd_header))) {
      log_message("[FCI Sim Server] Failed to read TCP command header");
      return;
    }
    
    char header_debug[256];
    sprintf(header_debug, "[FCI Sim Server] TCP Command: command=%u, command_id=%u, size=%u", 
            static_cast<uint32_t>(cmd_header.command), cmd_header.command_id, cmd_header.size);
    log_message(header_debug);
    
    // Dispatch command based on type
    switch (static_cast<protocol::Command>(cmd_header.command)) {
      case protocol::Command::kSetCollisionBehavior:
        handle_set_collision_behavior_command(cmd_header);
        break;
        
      case protocol::Command::kMove:
        handle_move_command(cmd_header);
        // Move command is now handled by the background UDP state thread
        break;
        
      case protocol::Command::kStopMove:
        handle_stop_move_command(cmd_header);
        break;
        
      default:
        // For unimplemented commands, send generic success response
        handle_generic_command(cmd_header);
        break;
    }
  }
}

void FrankaFciSimServer::handle_set_collision_behavior_command(const protocol::CommandHeader& header) {
  log_message("[FCI Sim Server] Handling SetCollisionBehavior command");
  
  // Read payload (collision parameters)
  if (header.size > sizeof(header)) {
    size_t payload_size = header.size - sizeof(header);
    std::vector<uint8_t> payload(payload_size);
    
    if (!read_exact(tcp_client_fd_, payload.data(), payload_size)) {
      log_message("[FCI Sim Server] Failed to read SetCollisionBehavior payload");
      return;
    }
    
    // We don't actually process the collision parameters in simulation
    log_message("[FCI Sim Server] SetCollisionBehavior parameters received (ignored in simulation)");
  }
  
  // Send success response
  protocol::CommandHeader response_header(
    protocol::Command::kSetCollisionBehavior,
    header.command_id,
    sizeof(protocol::CommandHeader) + sizeof(protocol::SetCollisionBehavior::Response)
  );
  
  protocol::SetCollisionBehavior::Response response(protocol::SetCollisionBehavior::Status::kSuccess);
  
  if (write_exact(tcp_client_fd_, &response_header, sizeof(response_header)) &&
      write_exact(tcp_client_fd_, &response, sizeof(response))) {
    log_message("[FCI Sim Server] Sent SetCollisionBehavior success response");
  } else {
    log_message("[FCI Sim Server] Failed to send SetCollisionBehavior response");
  }
}

void FrankaFciSimServer::handle_move_command(const protocol::CommandHeader& header) {
  log_message("[FCI Sim Server] Handling Move command");
  
  // Parse Move::Request
  if (header.size > sizeof(header)) {
    size_t payload_size = header.size - sizeof(header);
    
    if (payload_size >= sizeof(protocol::Move::Request)) {
      std::vector<uint8_t> move_request_data(sizeof(protocol::Move::Request));
      if (read_exact(tcp_client_fd_, move_request_data.data(), sizeof(protocol::Move::Request))) {
        const protocol::Move::Request* move_request = 
          reinterpret_cast<const protocol::Move::Request*>(move_request_data.data());
        
        requested_controller_mode_ = move_request->controller_mode;
        requested_motion_generator_mode_ = move_request->motion_generator_mode;
        
        char mode_msg[256];
        sprintf(mode_msg, "[FCI Sim Server] Move request: controller_mode=%d, motion_mode=%d", 
                (int)requested_controller_mode_, (int)requested_motion_generator_mode_);
        log_message(mode_msg);
        
        // Notify main process about mode change
        if (mode_change_handler_) {
          mode_change_handler_(requested_controller_mode_, requested_motion_generator_mode_);
        }
        
        // Read any remaining payload
        if (payload_size > sizeof(protocol::Move::Request)) {
          size_t remaining = payload_size - sizeof(protocol::Move::Request);
          std::vector<uint8_t> remaining_data(remaining);
          read_exact(tcp_client_fd_, remaining_data.data(), remaining);
        }
      }
    }
  }
  
  // Send Move::Response
  protocol::CommandHeader response_header(
    protocol::Command::kMove,
    header.command_id,
    sizeof(protocol::CommandHeader) + sizeof(protocol::Move::Response)
  );
  
  protocol::Move::Response response(protocol::Move::Status::kMotionStarted);
  
  if (write_exact(tcp_client_fd_, &response_header, sizeof(response_header)) &&
      write_exact(tcp_client_fd_, &response, sizeof(response))) {
    log_message("[FCI Sim Server] Sent Move success response - starting motion");
  } else {
    log_message("[FCI Sim Server] Failed to send Move response");
    return;
  }
  
  // IMPORTANT: We're already sending UDP states in background after Connect
  // Now we need to upgrade to active control mode that handles commands
  log_message("[FCI Sim Server] Move command processed, upgrading to active control mode");
  
  // Mark that we're now in control mode (the background thread will pick this up)
  control_mode_active_ = true;
}

void FrankaFciSimServer::handle_stop_move_command(const protocol::CommandHeader& header) {
  log_message("[FCI Sim Server] Handling StopMove command");
  
  // Read any payload
  if (header.size > sizeof(header)) {
    size_t payload_size = header.size - sizeof(header);
    std::vector<uint8_t> payload(payload_size);
    read_exact(tcp_client_fd_, payload.data(), payload_size);
  }
  
  // Send success response
  protocol::CommandHeader response_header(
    protocol::Command::kStopMove,
    header.command_id,
    sizeof(protocol::CommandHeader) + sizeof(protocol::StopMove::Response)
  );
  
  protocol::StopMove::Response response(protocol::StopMove::Status::kSuccess);
  
  if (write_exact(tcp_client_fd_, &response_header, sizeof(response_header)) &&
      write_exact(tcp_client_fd_, &response, sizeof(response))) {
    log_message("[FCI Sim Server] Sent StopMove success response");
  } else {
    log_message("[FCI Sim Server] Failed to send StopMove response");
  }
}

void FrankaFciSimServer::handle_generic_command(const protocol::CommandHeader& header) {
  char msg[256];
  sprintf(msg, "[FCI Sim Server] Handling generic command %u (not implemented)", static_cast<uint32_t>(header.command));
  log_message(msg);
  
  // Read any payload
  if (header.size > sizeof(header)) {
    size_t payload_size = header.size - sizeof(header);
    std::vector<uint8_t> payload(payload_size);
    read_exact(tcp_client_fd_, payload.data(), payload_size);
  }
  
  // Send generic success response
  protocol::CommandHeader response_header(
    static_cast<protocol::Command>(header.command),
    header.command_id,
    sizeof(protocol::CommandHeader) + 1 // Minimal response
  );
  
  uint8_t success_status = 0; // Success status
  
  if (write_exact(tcp_client_fd_, &response_header, sizeof(response_header)) &&
      write_exact(tcp_client_fd_, &success_status, sizeof(success_status))) {
    log_message("[FCI Sim Server] Sent generic success response");
  } else {
    log_message("[FCI Sim Server] Failed to send generic response");
  }
}

void FrankaFciSimServer::udp_control_loop() {
  log_message("[FCI Sim Server] UDP control loop started");
  
  // Debug: Print UDP client address
  char addr_debug[256];
  sprintf(addr_debug, "[FCI Sim Server] UDP client address: %s:%d", 
          inet_ntoa(udp_client_addr_.sin_addr), ntohs(udp_client_addr_.sin_port));
  log_message(addr_debug);
  
  // Start sending robot state immediately at 1kHz
  log_message("[FCI Sim Server] Starting continuous robot state transmission...");
  
  int loop_count = 0;
  auto last_state_time = std::chrono::steady_clock::now();
  
  while (running_ && udp_client_ready_) {
    auto current_time = std::chrono::steady_clock::now();
    
    // Send robot state at ~1kHz
    if (std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_state_time).count() >= 1000) {
      if (state_provider_) {
        protocol::RobotState state = state_provider_();
        state.message_id = message_id_++;
        state.robot_mode = protocol::RobotMode::kMove; // Set to Move mode for active control
        
        // Set appropriate modes based on requested motion type
        switch (requested_controller_mode_) {
          case protocol::Move::ControllerMode::kJointImpedance:
            state.controller_mode = protocol::ControllerMode::kJointImpedance;
            break;
          case protocol::Move::ControllerMode::kCartesianImpedance:
            state.controller_mode = protocol::ControllerMode::kCartesianImpedance;
            break;
          case protocol::Move::ControllerMode::kExternalController:
            state.controller_mode = protocol::ControllerMode::kExternalController;
            break;
          default:
            state.controller_mode = protocol::ControllerMode::kExternalController;
            break;
        }
        
        switch (requested_motion_generator_mode_) {
          case protocol::Move::MotionGeneratorMode::kJointPosition:
            state.motion_generator_mode = protocol::MotionGeneratorMode::kJointPosition;
            break;
          case protocol::Move::MotionGeneratorMode::kJointVelocity:
            state.motion_generator_mode = protocol::MotionGeneratorMode::kJointVelocity;
            break;
          case protocol::Move::MotionGeneratorMode::kCartesianPosition:
            state.motion_generator_mode = protocol::MotionGeneratorMode::kCartesianPosition;
            break;
          case protocol::Move::MotionGeneratorMode::kCartesianVelocity:
            state.motion_generator_mode = protocol::MotionGeneratorMode::kCartesianVelocity;
            break;
          default:
            state.motion_generator_mode = protocol::MotionGeneratorMode::kJointPosition;
            break;
        }
        
        state.control_command_success_rate = 1.0;
        state.errors.fill(false);
        state.reflex_reason.fill(false);
        
        ssize_t sent = sendto(udp_socket_fd_, &state, sizeof(state), 0,
                             (struct sockaddr*)&udp_client_addr_, sizeof(udp_client_addr_));
        
        last_state_time = current_time;
        
        // Debug every 1000 loops (every ~1 second)
        if (loop_count % 1000 == 0) {
          char state_msg[256];
          sprintf(state_msg, "[FCI Sim Server] Sent robot state #%d: msg_id=%lu, sent=%ld bytes", 
                  loop_count, state.message_id, sent);
          log_message(state_msg);
        }
      }
    }
    
    // Check for incoming UDP commands (non-blocking)
    protocol::RobotCommand command{};
    sockaddr_in from_addr{};
    socklen_t from_len = sizeof(from_addr);
    
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100; // 100μs timeout (non-blocking)
    setsockopt(udp_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    ssize_t received = recvfrom(udp_socket_fd_, &command, sizeof(command), 0,
                               (struct sockaddr*)&from_addr, &from_len);
    
    if (received == sizeof(command)) {
      // SUCCESS: Process the command
      char success_msg[256];
      sprintf(success_msg, "[FCI Sim Server] *** RECEIVED UDP COMMAND *** from %s:%d, msg_id=%lu", 
              inet_ntoa(from_addr.sin_addr), ntohs(from_addr.sin_port), command.message_id);
      log_message(success_msg);
      
      if (command_handler_) {
        command_handler_(command);
      }
      
      // Send immediate response state
      if (state_provider_) {
        protocol::RobotState response_state = state_provider_();
        response_state.message_id = command.message_id + 1;
        response_state.robot_mode = protocol::RobotMode::kMove;
        response_state.controller_mode = protocol::ControllerMode::kExternalController;
        response_state.motion_generator_mode = protocol::MotionGeneratorMode::kJointPosition;
        response_state.control_command_success_rate = 1.0;
        response_state.errors.fill(false);
        response_state.reflex_reason.fill(false);
        
        ssize_t sent = sendto(udp_socket_fd_, &response_state, sizeof(response_state), 0,
                             (struct sockaddr*)&from_addr, sizeof(from_addr));
        
        char response_msg[256];
        sprintf(response_msg, "[FCI Sim Server] Sent UDP response: cmd_id=%lu -> state_id=%lu", 
                command.message_id, response_state.message_id);
        log_message(response_msg);
        
        message_id_ = response_state.message_id;
      }
    } else if (received > 0) {
      // Unexpected size - log occasionally
      if (loop_count % 1000 == 0) {
        char msg[256];
        sprintf(msg, "[FCI Sim Server] Unexpected UDP packet size: %ld (expected %lu)", 
                received, sizeof(command));
        log_message(msg);
      }
    }
    // received < 0 with timeout is normal, just continue
    
    loop_count++;
    
    // 1kHz timing
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

void FrankaFciSimServer::set_mode_change_handler(ModeChangeHandler handler) {
  mode_change_handler_ = std::move(handler);
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