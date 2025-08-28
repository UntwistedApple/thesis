#include <Arduino.h>
#include "VP230.hpp"

const char* const TAG_CAN = "CAN";

VP230::VP230(int act) {
    pinMode(act, OUTPUT);
    digitalWrite(act, LOW);
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_18, GPIO_NUM_17, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    this->ready = true;

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(TAG_CAN, "Installed TWAI driver");
    } else {
        ESP_LOGE(TAG_CAN, "Failed to install driver");
        ready = false;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        ESP_LOGI(TAG_CAN, "Driver started");
    } else {
        ESP_LOGE(TAG_CAN, "Failed to start driver");
        ready = false;
    }
}

bool VP230::send(uint32_t identifier, std::vector<uint8_t>& data, int timeout_ms) {
    // Configure message to transmit
    twai_message_t message = {
        // Message type and format settings
        .extd = 1,              // Standard vs extended format
        .rtr = 0,               // Data vs RTR frame
        .ss = 0,                // Whether the message is single shot (i.e., does not repeat on error)
        .self = 0,              // Whether the message is a self reception request (loopback)
        .dlc_non_comp = 0,      // DLC is less than 8
        // Message ID and payload
        .identifier = identifier,
        .data_length_code = 4,
        .data = *data.data(),
    };

    // Queue message for transmission
    if (twai_transmit(&message, pdMS_TO_TICKS(timeout_ms)) == ESP_OK) {
        ESP_LOGI(TAG_CAN, "Message queued for transmission");
        return true;
    } else {
        ESP_LOGE(TAG_CAN, "Failed to queue message for transmission");
        return false;
    }
}

bool VP230::receive(twai_message_t* message, int timeout) {
    // Wait for the message to be received
    if (twai_receive(message, pdMS_TO_TICKS(timeout)) == ESP_OK) {
        ESP_LOGI(TAG_CAN, "Message received");
        return true;
    } else {
        ESP_LOGW(TAG_CAN, "Failed to receive message in time");
        return false;
    }
}