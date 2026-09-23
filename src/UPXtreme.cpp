#include "UPXtreme.h"
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>

UPXtreme::UPXtreme(const std::string &teensy_IP, const std::string &interface,
                   int udp_port, int param_response_port, int n_bus_line,
                   int n_actuator, std::string board_name)
    : teensy_IP_(teensy_IP), n_bus_line_(n_bus_line), n_actuator_(n_actuator),
      udp_port_(udp_port), send_socket(io_context), receive_socket(io_context),
      param_response_socket(io_context), udp_port_param_response_(param_response_port),
      board_name_(board_name)
{
    // Find the network interface IP address
    asio::ip::address_v4 network_intf_address;
    struct ifaddrs *ifaddr, *ifa;
    char host[NI_MAXHOST];

    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        exit(EXIT_FAILURE);
    }

    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL) continue;
        if (ifa->ifa_addr->sa_family == AF_INET &&
            strcmp(ifa->ifa_name, interface.c_str()) == 0)
        {
            int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                                host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (s != 0) {
                printf("getnameinfo() failed: %s\n", gai_strerror(s));
                exit(EXIT_FAILURE);
            }
            network_intf_address = asio::ip::make_address_v4(host);
            break;
        }
    }
    freeifaddrs(ifaddr);

    if (network_intf_address.is_unspecified()) {
        std::cerr << "Failed to find the " << interface << " interface IP address" << std::endl;
        throw std::runtime_error("Failed to find the network_intf interface IP address");
    }

    // Bind sockets to the selected network interface
    send_socket.open(asio::ip::udp::v4());
    send_socket.bind(asio::ip::udp::endpoint(network_intf_address, 0));

    receive_socket.open(asio::ip::udp::v4());
    receive_socket.set_option(asio::socket_base::reuse_address(true));
    receive_socket.bind(asio::ip::udp::endpoint(network_intf_address, udp_port_));

    // Non-blocking + a manual poll/retry loop (in receive_thread below) so
    // that loop periodically re-checks stop_threads on its own, rather than
    // depending entirely on end()'s receive_socket.close() to unblock a
    // pending receive_from() from another thread — that's not reliably
    // guaranteed to work on Linux, and hangs forever specifically when the
    // Teensy on the other end is simply powered off/disconnected and never
    // sends anything at all (confirmed 2026-09-10: bridge.stop() hung
    // indefinitely under exactly this condition).
    //
    // NOTE: a kernel-level SO_RCVTIMEO (via setsockopt on native_handle())
    // was tried first and does NOT work — confirmed by isolated test that
    // asio's synchronous receive_from() simply ignores it and blocks
    // forever regardless. non_blocking() + explicit error_code + a short
    // sleep/retry (below) is the only approach that actually works.
    receive_socket.non_blocking(true);

    param_response_socket.open(asio::ip::udp::v4());
    param_response_socket.set_option(asio::socket_base::reuse_address(true));
    param_response_socket.bind(asio::ip::udp::endpoint(network_intf_address, udp_port_param_response_));
    // Same non-blocking approach as receive_socket above — SO_RCVTIMEO
    // doesn't work, so receiveParamResponse() below polls manually instead.
    param_response_socket.non_blocking(true);

    std::cout << "send_socket    bound to " << send_socket.local_endpoint() << std::endl;
    std::cout << "receive_socket bound to " << receive_socket.local_endpoint() << std::endl;
    std::cout << "param_response_socket bound to " << param_response_socket.local_endpoint() << std::endl;

    // B3: initialize sys_data_ using runtime config params instead of hardcoded
    // N_ODRIVE_CAN1 / N_ODRIVE_CAN2 defines. Each bus carries n_actuator_ motors.
    sys_data_ = std::make_shared<SystemDataContainer>();
    for (int i = 0; i < n_bus_line_; ++i)
        sys_data_->add(SystemData<2>());  // 2 = N_actuator_per_CAN_bus_line
}

void UPXtreme::start()
{
    std::cout << "Starting UPXtreme threads for " << board_name_ << "..." << std::endl;

    receive_thread = std::thread([&]() {
        try {
            std::cout << "[" << board_name_ << "] Receive thread started." << std::endl;
            std::vector<uint8_t> recv_buffer(sys_data_->dataSize());
            while (!stop_threads) {
                asio::ip::udp::endpoint client_endpoint;
                asio::error_code ec;
                size_t bytes_received = receive_socket.receive_from(
                    asio::buffer(recv_buffer), client_endpoint, 0, ec);

                if (ec == asio::error::would_block) {
                    // Nothing available right now — expected, frequent,
                    // steady-state behavior on a non-blocking socket (see
                    // the constructor). Sleep briefly so this doesn't spin
                    // the CPU, then re-check stop_threads. Real packets
                    // arrive far more often than this during normal
                    // operation, so this sleep is rarely on the hot path.
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                if (ec) {
                    if (!stop_threads)
                        throw asio::system_error(ec);
                    break;
                }

                if (!stop_threads && bytes_received == sys_data_->dataSize())
                    handleUDPPacket(client_endpoint,
                                    {recv_buffer.begin(),
                                     recv_buffer.begin() + bytes_received});

#ifdef ENABLE_TIME_BENCHMARK
                receive_counter++;
#endif
            }
        } catch (const std::exception& e) {
            // Socket closed during shutdown is expected — only log unexpected errors
            if (!stop_threads)
                std::cerr << "[" << board_name_ << "] Receive thread exception: "
                          << e.what() << std::endl;
        }
        std::cout << "[" << board_name_ << "] Receive thread exiting." << std::endl;
    });

    send_thread = std::thread([&]() {
        static bool first_msg = true;
        while (!stop_threads) {
            if (sys_command_) {
                std::lock_guard<std::mutex> lock(command_mutex);
                std::vector<uint8_t> serialized_data = sys_command_->serializeWithHeader();
                sendToTeensy(serialized_data, serialized_data.size());
            } else if (first_msg) {
                std::cout << "[" << board_name_ << "] sys_command_ not yet initialized.\n";
                first_msg = false;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(30));
#ifdef ENABLE_TIME_BENCHMARK
            send_counter++;
#endif
        }
        std::cout << "[" << board_name_ << "] Send thread exiting." << std::endl;
    });

#ifdef ENABLE_TIME_BENCHMARK
    benchmark_thread = std::thread([&]() {
        while (!stop_threads) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            uint32_t rc = receive_counter.exchange(0);
            uint32_t sc = send_counter.exchange(0);
            std::cout << "\n--- UDP Benchmark [" << board_name_ << "] ---\n"
                      << "Avg receive frequency: " << rc / 5.0 << " Hz\n"
                      << "Avg send frequency:    " << sc / 5.0 << " Hz\n";
        }
    });
#endif
}

void UPXtreme::sendToTeensy(const std::vector<uint8_t> &data, const int data_size)
{
    std::vector<uint8_t> padded_data(data);
    padded_data.resize(data_size, 0);

    uint8_t crc_value = calculate_crc8(padded_data.data(), padded_data.size());
    std::vector<uint8_t> packet(padded_data);
    packet.push_back(crc_value);

    size_t bytes_sent = send_socket.send_to(
        asio::buffer(packet),
        udp::endpoint(asio::ip::make_address(teensy_IP_), udp_port_));

    if (bytes_sent != packet.size())
        printf("Failed to send complete packet: sent %zu of %zu bytes\n",
               bytes_sent, packet.size());
}

void UPXtreme::handleUDPPacket(const udp::endpoint &client_endpoint,
                                const std::vector<uint8_t> &data)
{
    // A3: lock data_mutex_ while writing sys_data_ so the control loop's
    // getPosEstimate() / getVelEstimate() calls don't race against this write.
    std::lock_guard<std::mutex> lock(data_mutex_);
    bool success = sys_data_->deserialize(data);

    static int packet_count = 0;
    packet_count++;
    if (!success && packet_count % 100 == 0)
        std::cout << "Warning: deserialization failed for packet #" << packet_count << std::endl;
}

bool UPXtreme::receiveParamResponse(ParamResponse &out, int timeout_ms)
{
    // Non-blocking + manual poll/retry, deliberately not a kernel-level
    // SO_RCVTIMEO: confirmed by isolated test (2026-09-10) that asio's
    // synchronous receive_from() ignores that socket option entirely and
    // blocks forever regardless — see the constructor's non_blocking(true)
    // call and the same finding for receive_socket/receive_thread. This is
    // a rare, synchronous, one-shot diagnostic call, not part of the 500Hz
    // receive_thread, so a short sleep between polls here is fine.
    uint8_t buffer[ParamResponse::wireSize()];
    asio::ip::udp::endpoint sender;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        asio::error_code ec;
        size_t bytes_received = param_response_socket.receive_from(
            asio::buffer(buffer), sender, 0, ec);

        if (!ec) {
            if (bytes_received != ParamResponse::wireSize())
                return false;
            out = ParamResponse::unpack(buffer);
            return true;
        }
        if (ec != asio::error::would_block)
            return false; // a real socket error, not just "nothing yet"

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false; // timed out
}
