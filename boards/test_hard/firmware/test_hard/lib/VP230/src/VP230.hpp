#pragma once

#include <vector>
#include <driver/twai.h>

class VP230 {


public:
    bool ready;

    VP230(int act);

    bool send(uint32_t identifier, std::vector<uint8_t>& data, int timeout);
    bool receive(twai_message_t* message_bfr, int timeout);
};