/*!
 * \file UDPBenchmark.cpp
 * \brief UDP round-trip benchmark between a PC and a Teensy 4.1.
 *
 * \details
 * This test measures end-to-end UDP latency, jitter, effective send/receive rates,
 * and packet loss while exercising a simple echo path between the PC (this program)
 * and a Teensy firmware.
 *
 * **On-wire packet format**
 * - `BenchmarkPacket` is serialized as:
 *   - 4-byte `sequence_number` in **network byte order** (big-endian)
 *   - 64-byte payload (opaque)
 * - `sizeof(BenchmarkPacket)` is expected to be 68 bytes.
 *
 * **PC behavior (this file)**
 * - A **sender thread** continuously transmits `BenchmarkPacket`s with a monotonically
 *   increasing 32-bit sequence number at a configurable cadence
 *   (see `std::this_thread::sleep_for(...)`).
 *   The send timestamp for each sequence is stored in `send_timestamps`.
 * - A **receiver thread** deserializes echoes from the Teensy and:
 *   - Runs at a much higher frequency than the sender thread because the Teensy is sending
 *     sending one UDP message per Teensy loop even if it hasn't received a new input message.
 *     In other words, the Teensy will keep sending duplicate of the latest message until
 *     it receives a new message from the PC.
 *   - If the sequence exists in `send_timestamps`, computes RTT and stores it.
 *   - Tracks packet loss using wrap-safe serial arithmetic:
 *     \code
 *     // signed distance on the modulo-2^32 ring
 *     diff = (int32_t)(pkt_seq - expected_seq);
 *     if (diff > 0)        // we are ahead -> missed 'diff' packets
 *       packet_loss += diff, expected_seq = pkt_seq + 1;
 *     else if (diff == 0)  // exactly expected
 *       expected_seq = pkt_seq + 1;
 *     else                 // diff < 0 -> out-of-order/duplicate, ignore
 *     \endcode
 *   - This treats the 32-bit sequence space as circular; no magic thresholds are used.
 *     Out-of-order or duplicate packets do **not** inflate loss.
 * - A **stats thread** prints every 5 seconds:
 *   - Avg / min / max latency and jitter (stddev) over the window
 *   - Packet loss count (sum of forward gaps in the window)
 *   - Send / receive frequencies (Hz) from atomic counters
 *   - Round-trip frequency = number of unique sequences that completed RTT in the window
 *   All counters are windowed (reset each print).
 *
 * **Teensy behavior (companion firmware)**
 * - Listens on UDP port 8000, parses `BenchmarkPacket`s, and **echoes** packets back to the PC.
 * - Depending on firmware configuration, the Teensy may:
 *   - Echo **once per new receive** (1:1 request/response), or
 *   - Free-run and repeatedly send the **most recently received** packet between receives.
 *     In the latter case the PC’s receive rate can exceed its send rate; RTT counting
 *     still deduplicates by sequence.
 * - The Teensy should call `Ethernet.loop()` regularly and fully consume any incoming datagrams
 *   to avoid RX stalls; an optional watchdog can re-init the UDP socket on prolonged RX idle.
 *
 * **Assumptions / caveats**
 * - UDP provides no reliability or ordering; bursts may reorder or drop.
 * - The sequence increments by 1 per send; wrap-around at 2^32 is expected and handled.
 * - The Teensy echoes bytes verbatim; the PC owns endianness conversion.
 * - If pushing very high PPS, consider increasing socket buffer sizes on the PC and/or
 *   reducing the send cadence during bring-up.
 *
 * **Interpreting the numbers**
 * - \b SendHz: PC transmit rate (how fast we push packets out).
 * - \b RecvHz: PC receive rate (how fast echoes arrive; can be higher than SendHz if the
 *   Teensy free-runs echoes).
 * - \b RTT\_Hz: rate of **unique** round-trips completed (deduped by sequence).
 * - \b PacketLoss: count of missing sequences inferred from forward jumps (reordering
 *   does not contribute).
 */


#include <unordered_map>
#include <unordered_set>
#include <numeric>

#define UPXTREME_i14

#include "UPXtreme.h"

struct BenchmarkPacket {
    uint32_t sequence_number;
    std::array<uint8_t, 64> payload;

    std::vector<uint8_t> serialize() const {
        std::vector<uint8_t> out;
        out.reserve(4 + payload.size());
        uint32_t seq_net = htonl(sequence_number);
        out.insert(out.end(), reinterpret_cast<uint8_t*>(&seq_net),
                              reinterpret_cast<uint8_t*>(&seq_net) + 4);
        out.insert(out.end(), payload.begin(), payload.end());
        return out;
    }

    static BenchmarkPacket deserialize(const std::vector<uint8_t>& in) {
        BenchmarkPacket pkt{};
        if (in.size() >= 4 + pkt.payload.size()) {
            uint32_t seq_net;
            memcpy(&seq_net, in.data(), 4);
            pkt.sequence_number = ntohl(seq_net);
            memcpy(pkt.payload.data(), in.data() + 4, pkt.payload.size());
        }
        return pkt;
    }
};

class UDPBenchmark : public UPXtreme {
public:
    UDPBenchmark(const std::string &teensy_IP, const std::string &interface, int udp_port, int n_bus_line, int n_actuator, std::string board_name)
    : UPXtreme(teensy_IP, interface, udp_port, n_bus_line, n_actuator, board_name)
    {
        start();
    }

    virtual void start();

    // Returns signed distance a - b in modulo-2^32 space.
    static inline int32_t seq_diff(uint32_t a, uint32_t b) {
        return static_cast<int32_t>(a - b);
    }

    
private:
    std::atomic<uint32_t> sequence_counter{0};
    std::atomic<uint32_t> received_packet_count{0};
    std::atomic<uint32_t> sent_packet_count{0};
    std::unordered_set<uint32_t> rtt_completed_packets;
    std::mutex benchmark_mutex;
    std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> send_timestamps;
    std::vector<double> latencies_ms;
    uint32_t expected_sequence = 0;
    uint32_t packet_loss_count = 0;
};

void UDPBenchmark::start()
{
    std::thread([&]() {
        while (true) {
            BenchmarkPacket pkt;
            pkt.sequence_number = sequence_counter++;
            {
                std::lock_guard<std::mutex> lock(benchmark_mutex);
                send_timestamps[pkt.sequence_number] = std::chrono::steady_clock::now();
            }
            std::vector<uint8_t> serialized = pkt.serialize();
            try {
            send_socket.send_to(asio::buffer(serialized), udp::endpoint(asio::ip::make_address(teensy_IP_), udp_port_));

            // Optional: print out TX parcel
            /*auto &d = serialized;
            printf("TX seq=%u  first8:", pkt.sequence_number);
            for (int i=0;i<8;i++) printf(" %02X", d[i]);
            printf("\n");*/

            } catch (const std::system_error &e) {
                PRINTLN("Send failed: ", e.what(), " [teensy_IP: ", teensy_IP_, "][udp_port: ", udp_port_, "]");
            }
            sent_packet_count++;
            std::this_thread::sleep_for(std::chrono::microseconds(25));
        }
    }).detach();

    std::thread([&]() {
        while (true) {
            std::vector<uint8_t> recv_buffer(1024);
            udp::endpoint sender_endpoint;
            size_t len = receive_socket.receive_from(asio::buffer(recv_buffer), sender_endpoint);

            // Optional: print out RX buffer
            /*printf("RX len=%zu from %s:%u  first16:", len, sender_endpoint.address().to_string().c_str(), sender_endpoint.port());
            for (size_t i=0;i<std::min<size_t>(len,16);++i) printf(" %02X", recv_buffer[i]);
            printf("\n");*/
            
            recv_buffer.resize(len);

            BenchmarkPacket pkt = BenchmarkPacket::deserialize(recv_buffer);
            // Optional: print out RX sequence_number
            //printf("seq=%u\n", pkt.sequence_number);

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

                static bool first_packet_recv = false;

                if (!first_packet_recv) {
                    expected_sequence = pkt.sequence_number + 1;
                    first_packet_recv = true;
                } else {
                    int32_t diff = seq_diff(pkt.sequence_number, expected_sequence);
                    if (diff > 0) {
                        // pkt is ahead -> we missed 'diff' packets
                        packet_loss_count += static_cast<uint32_t>(diff);
                        expected_sequence = pkt.sequence_number + 1;  // advance to next
                    } else if (diff == 0) {
                        // exactly the expected one
                        expected_sequence = pkt.sequence_number + 1;
                    } else {
                        // diff < 0 : out-of-order or duplicate; don't change expected_sequence
                        // TODO(@nicholasadr): track?
                    } 
                }
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
                packet_loss_count = 0;
            }
        }
    }).detach();
}


int main() {
    SystemConfig config;
    UDPBenchmark benchmark(config.teensy_IP[0],
                           config.PC_network_interface_name,
                           config.udp_port_PC_teensy[0],
                           config.N_CAN_bus_lines_per_teensy[0],
                           config.N_actuator_per_CAN_bus_line,
                           "UPXtreme");
    std::this_thread::sleep_for(std::chrono::minutes(10));
    return 0;
}
