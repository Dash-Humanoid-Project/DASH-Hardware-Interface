#define TEENSY_4_1

#include <QNEthernet.h>

using namespace qindesign::network;

IPAddress staticIP{10, 176, 32, 33};
IPAddress subnetMask{255, 255, 255, 0};
IPAddress gateway{10, 176, 32, 1};

constexpr uint16_t teensy_udp_port_listening = 8000;
constexpr uint16_t PC_udp_port_listening = 8000;

EthernetUDP udp;

struct BenchmarkPacket {
    uint32_t sequence_number;
    uint8_t payload[64];
};

BenchmarkPacket recv_packet;
IPAddress senderIP;
uint16_t senderPort;
bool first_packet_recv = false;

void printIPAddress()
{
    IPAddress ip = Ethernet.localIP();
    printf("    Local IP     = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.subnetMask();
    printf("    Subnet mask  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.broadcastIP();
    printf("    Broadcast IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.gatewayIP();
    printf("    Gateway      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.dnsServerIP();
    printf("    DNS          = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
}

void setup()
{
    Serial.begin(115200);

    // Wait for up to 3 seconds for the serial port to be opened on the PC side.
    // If no PC connects, continue anyway.
    for (int i = 0; i < 30 && !Serial; ++i) {
      delay(100);
    }
    delay(200);

    if (!setupEthernetWithStaticIP()) {
      Serial.println("Ethernet failed to initialize");
      while (true); // spin indefinitely
    };
    delay(2000); // wait for Ethernet confirmation
}

bool setupEthernetWithStaticIP()
{
    // Fetch MAC address out of the Teensy
    uint8_t mac[6];
    Ethernet.macAddress(mac);
    printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);


    // Show whether a cable is plugged in or not
    Ethernet.onLinkState([](bool state)
    {
      if (state)
        digitalWrite(LED_BUILTIN, HIGH);
      else
        digitalWrite(LED_BUILTIN, LOW);

      printf("[Ethernet] Link %s\r\n", state ? "ON" : "OFF");
    });

    // Static IP
    printf("Starting Ethernet with static IP...\r\n");
    if (!Ethernet.begin(staticIP, subnetMask, gateway)) {
      printf("Failed to start Ethernet\r\n");
      return false;
    }

    printf("Ethernet speed: %d\r\n", Ethernet.linkSpeed());

    printIPAddress();

    if (!udp.begin(teensy_udp_port_listening))
    {
      printf("Failed udp.begin\n");
      return false;
    }

    return true;
}

void loop()
{
    parseAndProcessUDPPacket(); // receive UDP message from UP to Teensy
    sendUDPPacket(); // send UDP message from Teensy to UP
}

void parseAndProcessUDPPacket()
{
    int size = udp.parsePacket();
    if (size == sizeof(BenchmarkPacket))
    {
        senderIP = udp.remoteIP();     // capture sender for echo
        senderPort = udp.remotePort();
        udp.read((char *)&recv_packet, sizeof(BenchmarkPacket));
        first_packet_recv = true;
    }
}

void sendUDPPacket()
{
    if (!first_packet_recv)
        return;

    int result = udp.beginPacket(senderIP, senderPort);
    if (!result) {
        printf("beginPacket failed\n");
        return;
    }

    udp.write((const uint8_t *)&recv_packet, sizeof(BenchmarkPacket));

    if (!udp.endPacket()) {
        printf("Error sending UDP from Teensy\n");
    }
}
