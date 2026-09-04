#include "UPXtreme.h"
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

UPXtreme::UPXtreme(const std::string &teensy_IP, const std::string &interface,
                   int udp_port, int n_bus_line, int n_actuator, std::string board_name)
    : teensy_IP_(teensy_IP), n_bus_line_(n_bus_line), n_actuator_(n_actuator),
      udp_port_(udp_port), send_socket(io_context), receive_socket(io_context),
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

    std::cout << "send_socket    bound to " << send_socket.local_endpoint() << std::endl;
    std::cout << "receive_socket bound to " << receive_socket.local_endpoint() << std::endl;

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
                size_t bytes_received = receive_socket.receive_from(
                    asio::buffer(recv_buffer), client_endpoint);

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
