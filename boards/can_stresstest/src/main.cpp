#include <Arduino.h>
#include "VP230.hpp"

VP230 can_transceiver(GPIO_NUM_15, GPIO_NUM_14, GPIO_NUM_9);
packet_t packet;

void setup() {
}

void loop() {
  packet = VP230::prepare_packet(0, MTYPE_SPAM, 0b1111, 0, 0);
  can_transceiver.send_packet(packet);

}