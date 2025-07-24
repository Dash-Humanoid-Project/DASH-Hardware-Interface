
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <asio.hpp>

using asio::ip::udp;

struct BenchmarkPacket {
    uint32_t sequence_number;
    std::array<uint8_t, 64> payload;

    std::vector<uint8_t> serialize() const {
        std::vector<uint8_t> data(sizeof(BenchmarkPacket));
        memcpy(data.data(), this, sizeof(BenchmarkPacket));
        return data;
    }

    static BenchmarkPacket deserialize(const std::vector<uint8_t>& data) {
        BenchmarkPacket pkt{};
        if (data.size() >= sizeof(BenchmarkPacket)) {
            memcpy(&pkt, data.data(), sizeof(BenchmarkPacket));
        }
        return pkt;
    }
};

class UDPBenchmark {
public:
    UDPBenchmark(const std::string& teensy_ip, int port)
        : teensy_ip_(teensy_ip), udp_port_(port),
          send_socket(io_context), receive_socket(io_context) {

        send_socket.open(udp::v4());
        send_socket.bind(udp::endpoint(udp::v4(), 0));

        receive_socket.open(udp::v4());
        receive_socket.bind(udp::endpoint(udp::v4(), udp_port_));

        start();
    }

private:
    asio::io_context io_context;
    udp::socket send_socket;
    udp::socket receive_socket;
    std::string teensy_ip_;
    int udp_port_;

    std::atomic<uint32_t> sequence_counter{0};
    std::atomic<uint32_t> received_packet_count{0};
    std::atomic<uint32_t> sent_packet_count{0};
    std::unordered_set<uint32_t> rtt_completed_packets;
    std::mutex benchmark_mutex;
    std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> send_timestamps;
    std::vector<double> latencies_ms;
    uint32_t expected_sequence = 0;
    uint32_t packet_loss_count = 0;

    void start() {
        std::thread([&]() {
            while (true) {
                BenchmarkPacket pkt;
                pkt.sequence_number = sequence_counter++;
                {
                    std::lock_guard<std::mutex> lock(benchmark_mutex);
                    send_timestamps[pkt.sequence_number] = std::chrono::steady_clock::now();
                }
                std::vector<uint8_t> serialized = pkt.serialize();
                send_socket.send_to(asio::buffer(serialized), udp::endpoint(asio::ip::make_address(teensy_ip_), udp_port_));
                sent_packet_count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }).detach();

        std::thread([&]() {
            while (true) {
                std::vector<uint8_t> recv_buffer(1024);
                udp::endpoint sender_endpoint;
                size_t len = receive_socket.receive_from(asio::buffer(recv_buffer), sender_endpoint);
                recv_buffer.resize(len);

                BenchmarkPacket pkt = BenchmarkPacket::deserialize(recv_buffer);
                auto now = std::chrono::steady_clock::now();
                double latency_ms = 0;
                received_packet_count++;

                {
                    std::lock_guard<std::mutex> lock(benchmark_mutex);
                    auto it = send_timestamps.find(pkt.sequence_number);
                    if (it != send_timestamps.end()) {
                        latency_ms = std::chrono::duration<double, std::milli>(now - it->second).count();
                        latencies_ms.push_back(latency_ms);
                        rtt_completed_packets.insert(pkt.sequence_number);
                        send_timestamps.erase(it);
                    }

                    if (pkt.sequence_number != expected_sequence) {
                        packet_loss_count += pkt.sequence_number - expected_sequence;
                    }
                    expected_sequence = pkt.sequence_number + 1;
                }
            }
        }).detach();

        std::thread([&]() {
            while (true) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                std::lock_guard<std::mutex> lock(benchmark_mutex);

                uint32_t recv_count = received_packet_count.exchange(0);
                uint32_t sent_count = sent_packet_count.exchange(0);
                uint32_t rtt_count = rtt_completed_packets.size();
                double recv_freq = recv_count / 5.0;
                double send_freq = sent_count / 5.0;
                double rtt_freq = rtt_count / 5.0;
                rtt_completed_packets.clear();

                if (!latencies_ms.empty()) {
                    double sum = std::accumulate(latencies_ms.begin(), latencies_ms.end(), 0.0);
                    double avg = sum / latencies_ms.size();
                    double min = *std::min_element(latencies_ms.begin(), latencies_ms.end());
                    double max = *std::max_element(latencies_ms.begin(), latencies_ms.end());
                    double jitter = std::sqrt(std::inner_product(latencies_ms.begin(), latencies_ms.end(), latencies_ms.begin(), 0.0) / latencies_ms.size() - avg * avg);

                    std::cout << "\n--- UDP Benchmark ---\n";
                    std::cout << "Avg latency: " << avg << " ms\n";
                    std::cout << "Min/Max latency: " << min << "/" << max << " ms\n";
                    std::cout << "Jitter (stddev): " << jitter << " ms\n";
                    std::cout << "Packet loss count: " << packet_loss_count << "\n";
                    std::cout << "Send frequency:    " << send_freq << " Hz\n";
                    std::cout << "Receive frequency: " << recv_freq << " Hz\n";
                    std::cout << "Round-trip frequency: " << rtt_freq << " Hz\n";
                    std::cout << "---------------------\n";

                    latencies_ms.clear();
                }
            }
        }).detach();
    }
};

int main() {
    UDPBenchmark benchmark("192.168.1.50", 8888);  // Replace with your Teensy IP and port
    std::this_thread::sleep_for(std::chrono::minutes(10));  // Run benchmark for 10 minutes
    return 0;
}
