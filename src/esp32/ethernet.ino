#include <ETH.h>
#include <SPI.h>

#define ETH_SO 17
#define ETH_SI 6
#define ETH_CLK 4
#define ETH_CS 44
#define ETH_INT 7
#define ETH_RST 5
#define ETH_PHY_ADDR 1
#define ETH_PHY_SPI_FREQ_MHZ 20

void ethernet_init(char *agent_ip, char *local_ip) {
  log_i("Initializing Ethernet...");

  SPI.begin(ETH_CLK, ETH_SO, ETH_SI);
  log_i("Initialized SPI.");

  if (ETH.begin(ETH_PHY_W5500,
                ETH_PHY_ADDR,
                ETH_CS,
                ETH_INT,
                ETH_RST,
                SPI,
                ETH_PHY_SPI_FREQ_MHZ)) {
    log_i("Initialized Ethernet.");
  } else {
    log_i("Failed to initialize Ethernet.");
  }

  log_i("Connecting Ethernet...");
  while (!ETH.linkUp()) {
    delay(100);
  }
  log_i("Link established.");

  IPAddress localIP;
  localIP.fromString(local_ip);

  IPAddress gateway;
  gateway.fromString(agent_ip);

  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns = gateway;

  if (!ETH.config(localIP, gateway, subnet, dns)) {
    log_i("Error: Static IP configuration failed!");
  } else {
    log_i("Static IP configured.");
  }

  log_i("Waiting for IP...");
  while (!ETH.hasIP()) {
    delay(50);
  }
  log_i("Got IP.");

  log_i("Ethernet Ready. IP: %s", ETH.localIP().toString(true));
}

static NetworkUDP udp_client;

bool eth_transport_open(struct uxrCustomTransport *transport) {
  struct micro_ros_agent_locator *locator = (struct micro_ros_agent_locator *)transport->args;
  udp_client.stop();
  udp_client.begin(locator->port);
  return true;
}

bool eth_transport_close(struct uxrCustomTransport *transport) {
  udp_client.stop();
  return true;
}

size_t eth_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode) {
  (void)errcode;
  struct micro_ros_agent_locator *locator = (struct micro_ros_agent_locator *)transport->args;

  udp_client.beginPacket(locator->address, locator->port);
  size_t sent = udp_client.write(buf, len);
  udp_client.endPacket();
  udp_client.flush();

  return sent;
}

size_t eth_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode) {
  (void)errcode;

  uint32_t start_time = millis();

  while (millis() - start_time < timeout && udp_client.parsePacket() == 0) {
    delay(1);
  }

  int readed = udp_client.read(buf, len);

  return (readed < 0) ? 0 : readed;
}

static inline void set_microros_ethernet_udp_transports(char *agent_ip, uint agent_port) {
  static struct micro_ros_agent_locator locator;
  locator.address.fromString(agent_ip);
  locator.port = agent_port;

  rmw_uros_set_custom_transport(
      false,
      (void *)&locator,
      eth_transport_open,
      eth_transport_close,
      eth_transport_write,
      eth_transport_read);
}
