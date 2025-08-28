#include <Arduino.h>
#include "VP230.hpp"

VP230 can = VP230(GPIO_NUM_32, GPIO_NUM_33);

std::vector<uint8_t> data = {1,2,3,4,5,6,7,8};
packet_t pckg;

void setup() {
  pckg.value = new uint8_t[8];

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!can.receive_packet(&pckg, 1000)) return;

  if (pckg.type == MTYPE_DIFF_V) {
    int res=0;
    memcpy(&res, pckg.value, 4);
    ESP_LOGI("CAN", "Received %s\t %3.3fkOhm, \tchanged by %5d\t from node %d, sensor %d", 
    VP230::get_message_name(pckg.type).c_str(), res/1000.0, pckg.change, pckg.node_id, pckg.sensor_id);  

    Serial.print(res);
    Serial.println();
  } else {
    double res=0;
    memcpy(&res, pckg.value, 8);
    ESP_LOGI("CAN", "Received %s\t %10fV, \tchanged by %5d\t from node %d, sensor %d", 
    VP230::get_message_name(pckg.type).c_str(), res, pckg.change, pckg.node_id, pckg.sensor_id);

    Serial.print(res, 10);
    Serial.print(",");
  }

  // can.send(0xff, data, 1000);
}
