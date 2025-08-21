#include "UPXtreme.h"
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>


UPXtreme::UPXtreme(const std::string &teensy_IP, const std::string &interface, int udp_port, int n_bus_line, int n_actuator, std::string board_name)
    : teensy_IP_(teensy_IP), n_bus_line_(n_bus_line), n_actuator_(n_actuator), udp_port_(udp_port),
      send_socket(io_context), receive_socket(io_context), board_name_(board_name)
{
    // Find the network interface IP address
    asio::ip::address_v4 network_intf_address;
    struct ifaddrs *ifaddr, *ifa;
    int family, s;
    char host[NI_MAXHOST];

    if (getifaddrs(&ifaddr) == -1)
    {
        perror("getifaddrs");
        exit(EXIT_FAILURE);
    }

    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL)
            continue;

        family = ifa->ifa_addr->sa_family;

        if (family == AF_INET && strcmp(ifa->ifa_name, interface.c_str()) == 0)
        {
            s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                            host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (s != 0)
            {
                printf("getnameinfo() failed: %s\n", gai_strerror(s));
                exit(EXIT_FAILURE);
            }
            network_intf_address = asio::ip::make_address_v4(host);
            break;
        }
    }

    freeifaddrs(ifaddr);

    if (network_intf_address.is_unspecified())
    {
        std::cerr << "Failed to find the " << interface << " interface IP address" << std::endl;
        throw std::runtime_error("Failed to find the network_intf interface IP address");
    }

    // Bind the sending socket to the network_intf interface
	// Binding to port 0 means the local sending port will be chosen automatically
    send_socket.open(asio::ip::udp::v4());
    send_socket.bind(asio::ip::udp::endpoint(network_intf_address, 0));

    // Bind the server socket to the network_intf interface
	// Bind receiving socket to the assigned UDP port for communication with Teensy
    receive_socket.open(asio::ip::udp::v4());
    receive_socket.bind(asio::ip::udp::endpoint(network_intf_address, udp_port_));

	std::cout << "send_socket    bound to " << send_socket.local_endpoint() << std::endl;
    std::cout << "receive_socket bound to " << receive_socket.local_endpoint() << std::endl;

    sys_data_ = std::make_shared<SystemDataContainer>();
    sys_data_->add(SystemData<N_ODRIVE_CAN1>());
    sys_data_->add(SystemData<N_ODRIVE_CAN2>());
    sys_data_->add(SystemData<N_ODRIVE_CAN3>());
}

void UPXtreme::start()
{
    // Start the server in a separate thread
    receive_thread = std::thread([&]()
    {
        std::vector<uint8_t> recv_buffer(sys_data_->dataSize());
		while (true) {
			asio::ip::udp::endpoint client_endpoint;
			size_t bytes_received = receive_socket.receive_from(asio::buffer(recv_buffer), client_endpoint);
			handleUDPPacket(client_endpoint, {recv_buffer.begin(), recv_buffer.begin() + bytes_received});

#ifdef ENABLE_TIME_BENCHMARK
            receive_counter++;
#endif
            }
	});

    send_thread = std::thread([&]()
	{
        while (true) {
            // Serialize the SystemCommand object
            if(sys_command_)
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                std::vector<uint8_t> serialized_data = sys_command_->serializeWithHeader();
                sendToTeensy(serialized_data, serialized_data.size());

            }
            else
                std::cout << "sys_command_ is not initialized\n";

            std::this_thread::sleep_for(std::chrono::microseconds(30));

#ifdef ENABLE_TIME_BENCHMARK
            send_counter++;
#endif
        }
    });

#ifdef ENABLE_TIME_BENCHMARK
    benchmark_thread = std::thread([&]()
    {
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(5));

            uint32_t receive_count = receive_counter.exchange(0);
            uint32_t send_count = send_counter.exchange(0);
            double receive_freq = receive_count / 5.0;
            double send_freq = send_count / 5.0;

            std::cout << "\n--- UDP Benchmark ---\n";
            std::cout << "Avg receive frequency: " << receive_freq << " Hz\n";
            std::cout << "Avg send frequency: " << send_freq << " Hz\n";
        }
    });
#endif
}

void UPXtreme::sendToTeensy(const std::vector<uint8_t> &data, const int data_size)
{
    // Pad or truncate the data to match the expected size
    std::vector<uint8_t> padded_data(data);
    padded_data.resize(data_size, 0);

    // Calculate CRC-8 for the payload
    uint8_t crc_value = calculate_crc8(padded_data.data(), padded_data.size());

    // Append the CRC value to the payload
    std::vector<uint8_t> packet(padded_data);
    packet.push_back(crc_value);

    // Send the data to the Teensy
	size_t bytes_sent = send_socket.send_to(asio::buffer(packet), udp::endpoint(asio::ip::make_address(teensy_IP_), udp_port_));

	if (bytes_sent != packet.size()) {
    	printf("Failed to send complete packet: Sent %zu out of %zu bytes\n", bytes_sent, packet.size());
	}
}

void UPXtreme::handleUDPPacket(const udp::endpoint &client_endpoint, const std::vector<uint8_t> &data)
{
    // Unpack the received data
    std::vector<uint8_t> data_list(data.begin(), data.end());

    sys_data_->deserialize(data_list);
}
