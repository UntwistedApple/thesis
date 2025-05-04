#include <Arduino.h>
#include "esp_system.h"
#include "ADS114.hpp"

#define PIN_NUM_MISO  GPIO_NUM_13
#define PIN_NUM_MOSI  GPIO_NUM_11
#define PIN_NUM_CLK   GPIO_NUM_12
#define PIN_NUM_CS    GPIO_NUM_10

#define ADS_SPI_HOST SPI2_HOST

const char* const TAG_SPI = "SPI";
const char* const TAG_ADC = "ADC";

DRAM_ATTR static const uint8_t ads_cmds[] = {
    /*************************
     *   Control Commands    *
     ************************/
    0x00,   // NOP
    0x02,   // Wake up from power down
    0x04,   // Power down
    0x06,   // Reset
    0x08,   // Start conversions
    0x0A,   // Stop conversions
    /*************************
     * Calibration Commands  *
     ************************/
    0x16,   // System offset calibration
    0x17,   // System gain calibration
    0x19,   // Self offset calibration
    /*************************
     *   Data Read Command   *
     ************************/
    0x12,   // Read data
    /*************************
     * Register R/W Commands *
     ************************/
    0x20,   // Register read. 5 lower bits should be register
    0x40    // Register write. 5 lower bits should be register
};

ADS114::ADS114(spi_host_device_t spi_device, gpio_num_t gpio_num_rdy):
    spi_device(spi_device),
    gpio_num_rdy(gpio_num_rdy),
    spi_handle() 
{
    esp_err_t ret = 0;
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,                    // No quad SPI used, so no wp (write-protect) pin needed
        .quadhd_io_num = -1                     // No quad SPI used, so no hd (hold) pin needed
    };
    spi_device_interface_config_t devcfg = {
        .command_bits = 8,
        .address_bits = 8,
        .mode = 1,                              //SPI mode 1
        .clock_speed_hz = SPI_MASTER_FREQ_8M,   //Clock out at 8 MHz
        .spics_io_num = PIN_NUM_CS,             //CS pin
        .queue_size = 4,                        //We want to be able to queue 4 transactions at a time
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(spi_device, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the ADS to the SPI bus
    ret = spi_bus_add_device(spi_device, &devcfg, &spi_handle);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG_SPI, "Bus ready");

    reset();

    bool check_successful = check_id();
    if (!check_successful) {
        while(1) {}
    }

    set_internal_reference(REFCON_INTERNAL_ON); // Enable internal reference
    select_reference(REFSEL_REF_INTERNAL);      // Select internal reference
    set_ref_bufs(REFP_BUF_DIS, REFN_BUF_DIS);

    delay(100);
}

void ADS114::start() {
    ESP_LOGI(TAG_ADC, "Starting conversion");
    spi_transaction_ext_t transaction_ext = {
        .base = {
            .flags = SPI_TRANS_VARIABLE_ADDR,
            .cmd = ads_cmds[START],
            .length = 0
        }
    };
    esp_err_t ret = spi_device_transmit(spi_handle, (spi_transaction_t*) &transaction_ext);
    ESP_ERROR_CHECK(ret);
}

int16_t ADS114::data_read() {
    ESP_LOGI(TAG_ADC, "Fetching conversion result");
    spi_transaction_ext_t transaction_ext = {
        .base = {
            .flags = 
                SPI_TRANS_VARIABLE_ADDR | 
                // SPI_TRANS_VARIABLE_CMD  |
                SPI_TRANS_USE_RXDATA,
            .cmd = ads_cmds[RDATA],
            .length = 16
        }
    };
    esp_err_t ret = spi_device_transmit(spi_handle, (spi_transaction_t*) &transaction_ext);
    ESP_ERROR_CHECK(ret);

    uint16_t result = transaction_ext.base.rx_data[0] << 8 | transaction_ext.base.rx_data[1];
    ESP_LOGD(TAG_ADC, "Got conversion result: 0x%X", result);

    return result;
}

uint8_t ADS114::reg_read(reg_name reg) {
    ESP_LOGI(TAG_SPI, "Reading from reg 0x%X", reg);
    spi_transaction_t transaction = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = (uint8_t) (ads_cmds[RREG] | reg),
        .addr = 0,
        .length = 8,
    };
    esp_err_t ret = spi_device_transmit(spi_handle, &transaction);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG_SPI, "Register read: 0x%X",transaction.rx_data[0]);
    return transaction.rx_data[0];
}

uint16_t ADS114::reg_read_2(reg_name reg) {
    ESP_LOGI(TAG_SPI, "Reading 2 regs starting from 0x%X", reg);
    spi_transaction_t transaction = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = (uint8_t) (ads_cmds[RREG] | reg),
        .addr = 1,
        .length = 16,
    };
    esp_err_t ret = spi_device_transmit(spi_handle, &transaction);
    ESP_ERROR_CHECK(ret);
    uint16_t res = *((uint16_t*)transaction.rx_data);
    ESP_LOGI(TAG_SPI, "Register read: 0x%X",res);
    return res;
}

void ADS114::reg_write(uint8_t reg, uint8_t data) {
    ESP_LOGI(TAG_SPI, "Writing to reg 0x%X data 0x%X", reg, data);
    spi_transaction_t transaction = {
        .flags = SPI_TRANS_USE_TXDATA,
        .cmd = (uint8_t) (ads_cmds[WREG] | reg),
        .addr = 0,
        .length = 8,
        .tx_data = {data, 0, 0, 0}
    };
    esp_err_t ret = spi_device_transmit(spi_handle, &transaction);
    ESP_ERROR_CHECK(ret);
    ESP_LOGD(TAG_SPI, "Write successful");
}

void ADS114::register_dump() {
    ESP_LOGI(TAG_ADC, "Dumping ADS register field");

    std::array<uint8_t, 18> result{};

    spi_transaction_t transaction = {
        .cmd = ads_cmds[RDATA],
        .length = 8*sizeof(result)      // 18 registers * 8 bit
    };
    esp_err_t ret = spi_device_transmit(spi_handle, (spi_transaction_t*) &transaction);
    ESP_ERROR_CHECK(ret);
}

bool ADS114::check_id() {
    ESP_LOGI(TAG_ADC, "Starting ADS ID check...");
    uint8_t id = reg_read(REG_ID); // Read ID register
    id &= 0x07;
    if (id != 0x04) {
        ESP_LOGE(TAG_ADC, "ADS ID is different than expected. Expected 0x04, got 0x%X", id);
        return false;
    }
    ESP_LOGI(TAG_ADC, "ADS ID check succesful");
    return true;
}

void ADS114::reset() {
    ESP_LOGI(TAG_ADC, "Resetting ADS");
    bool por = reg_read(REG_STATUS) & BIT(STATUS_FL_POR);
    
    if (por) {
        ESP_LOGI(TAG_ADC, "POR flag set, unsetting..");
        reg_write(REG_STATUS, 0);
        bool por = reg_read(REG_STATUS) & BIT(STATUS_FL_POR);
        if (por) {
            ESP_LOGE(TAG_ADC, "POR unset was unseccessful");
        } else {
            ESP_LOGI(TAG_ADC, "POR flag unset");
        }
    }

    spi_transaction_ext_t transaction_ext = {
        .base = {
            .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_VARIABLE_ADDR,
            .cmd = ads_cmds[RESET],
            .length = 0
        }
    };
    esp_err_t ret = spi_device_transmit(spi_handle, (spi_transaction_t*) &transaction_ext);
    ESP_ERROR_CHECK(ret);

    bool por_after = reg_read(REG_STATUS) & BIT(STATUS_FL_POR);
    
    if (por_after) {
        ESP_LOGI(TAG_ADC, "ADS reset successful");
    } else {
        ESP_LOGE(TAG_ADC, "ADS reset unsuccessful");
    }
}

double ADS114::digital_supply_voltage_read() {
    ESP_LOGI(TAG_ADC, "Reading digital supply voltage");
    set_sys_mon(SYS_MON_DVDD);
    start();

    // Wait for reading
    while (digitalRead(gpio_num_rdy)) {}
    int16_t result = data_read();
    double lsb_range = 5.0/(1<<16);
    double final = (double) result * lsb_range * 4;
    ESP_LOGD(TAG_ADC, "DVDD = 0x%X = %fV", result, final);
    return final;
}

double ADS114::analog_supply_voltage_read() {
    ESP_LOGI(TAG_ADC, "Reading analog supply voltage");
    set_sys_mon(SYS_MON_AVDD);
    start();

    // Wait for reading
    while (digitalRead(gpio_num_rdy)) {}
    int16_t result = data_read();
    double lsb_range = 5.0/(1<<16);
    double final = (double) result * lsb_range * 4;
    ESP_LOGD(TAG_ADC, "AVDD = 0x%X = %fV", result, final);
    return final;
}

void ADS114::set_internal_reference(refcon val) {
    ESP_LOGI(TAG_ADC, "Setting internal reference \"REFCON\" to 0x%X", val);
    uint8_t ref_reg_content = reg_read(REG_REF);

    // Set internal reference
    ref_reg_content &= ~REFCON_MASK;      // Unset all REFCON bits
    ref_reg_content |= val;

    // Write back
    reg_write(REG_REF, ref_reg_content);
    ESP_LOGD(TAG_ADC, "REF register set to 0x%X", ref_reg_content);
}

void ADS114::select_reference(refsel val) {
    /**
     * When enabling internal reference, both reference buffers should be disabled!
     */
    ESP_LOGI(TAG_ADC, "Selecting reference \"REFSEL\" to 0x%X", val);
    uint8_t ref_reg_content = reg_read(REG_REF);

    // Select reference
    ref_reg_content &= ~REFSEL_MASK;      // Unset all REFSEL bits
    ref_reg_content |= val;

    // Write back
    reg_write(REG_REF, ref_reg_content);
    ESP_LOGD(TAG_ADC, "REF register set to 0x%X", ref_reg_content);
}

void ADS114::set_ref_bufs(refp refp_val, refn refn_val) {
    ESP_LOGI(TAG_ADC, "Setting reference buffers to refp 0x%X and refn 0x%x", refp_val, refn_val);
    uint8_t ref_reg_content = reg_read(REG_REF);

    ref_reg_content &= ~(BIT(REFP_BUF) | BIT(REFN_BUF));      // Unset all buffer-relevant bits
    ref_reg_content |= refp_val | refn_val;

    reg_write(REG_REF, ref_reg_content);
    ESP_LOGD(TAG_ADC, "REF register set to 0x%X", ref_reg_content);
}

void ADS114::pga(pga_en pga_en_val, pga_gain pga_gain_val) {
    /**
     * @brief When bypassing the PGA, gain should be set to 1 (0b000)
     * 
     */

    // Both values give us the complete register content, therefore no 
    // previous content has to be preserved
    ESP_LOGI(TAG_ADC, "Setting PGA register to 0x%X", pga_en_val | pga_gain_val);
    reg_write(REG_PGA, pga_en_val | pga_gain_val);
}

void ADS114::inpmux(inpmux_muxp muxp_val, inpmux_muxn muxn_val) {
    ESP_LOGI(TAG_ADC, "Setting INPMUX register to 0x%X", muxp_val | muxn_val);
    reg_write(REG_INPMUX, muxp_val | muxn_val);
}

void ADS114::set_sys_mon(sys_mon sys_mon_val) {
    ESP_LOGI(TAG_ADC, "Setting SYS->sys_mon to 0x%X", sys_mon_val);
    uint8_t reg_content = reg_read(REG_SYS);

    reg_content &= ~SYS_MON_MASK;      // Unset all relevant bits
    reg_content |= sys_mon_val;

    reg_write(REG_SYS, reg_content);
    ESP_LOGD(TAG_ADC, "SYS register set to 0x%X", reg_content);
}

void ADS114::set_cal_samp(cal_samp cal_samp_val) {
    ESP_LOGI(TAG_ADC, "Setting SYS->cal_samp to 0x%X", cal_samp_val);
    uint8_t reg_content = reg_read(REG_SYS);

    reg_content &= ~CAL_SAMP_MASK;      // Unset all relevant bits
    reg_content |= cal_samp_val;

    reg_write(REG_SYS, reg_content);
    ESP_LOGD(TAG_ADC, "SYS register set to 0x%X", reg_content);
}

void ADS114::set_idacmag(imag imag_val) {
    ESP_LOGI(TAG_ADC, "Settin IDACMAG to 0x%X", imag_val);

    reg_write(REG_IDACMAG, imag_val);
    
    ESP_LOGD(TAG_ADC, "Set IDACMAG to 0x%X", imag_val);
}

void ADS114::set_idacmux(i1mux i1mux_val, i2mux i2mux_val) {
    ESP_LOGI(TAG_ADC, "Setting IDAC1 mux to 0x%X and IDAC2 mux to 0x%X", i1mux_val, i2mux_val);

    uint8_t reg_content = i1mux_val | i2mux_val;
    reg_write(REG_IDACMUX, reg_content);

    ESP_LOGD(TAG_ADC, "Set reg IDACMUX to 0x%X", reg_content);
}

void ADS114::conv_mode(datarate_mode mode) {
    ESP_LOGI(TAG_ADC, "Setting conversion mode to 0x%X", mode);
    uint8_t reg_content = reg_read(REG_DATARATE);

    reg_content &= ~BIT(DATARATE_MODE);
    reg_content |= mode;

    reg_write(REG_DATARATE, reg_content);
    ESP_LOGD(TAG_ADC, "DATARATE register set to 0x%X", reg_content);
}

void ADS114::set_datarate(datarate_mode mode_val, datarate_dr dr_val, datarate_clk clk_val) {
    uint8_t reg = BIT(4);
    reg |= mode_val | dr_val | clk_val;

    ESP_LOGI(TAG_ADC, "Setting DATARATE reg to 0x%X", reg);

    reg_write(REG_DATARATE, reg);
    ESP_LOGD(TAG_ADC, "Set DATARATE reg");
}

uint16_t ADS114::ofcal_read() {
    ESP_LOGD(TAG_ADC, "Reading OFCAL value");
    uint16_t ofcal_value = reg_read_2(REG_OFCAL0);
    ESP_LOGD(TAG_ADC, "Read OFCAL value of 0x%X", ofcal_value);

    return ofcal_value;
}

uint16_t ADS114::calibrate() {
    /**
     * @brief Self calibrates the ADS114 device, returns conversion result after calibration
     * 
     */

    ESP_LOGI(TAG_ADC, "Calibrating");

    spi_transaction_ext_t transaction_ext = {
        .base = {
            .flags = SPI_TRANS_VARIABLE_ADDR,
            .cmd = ads_cmds[SFOCAL],
            .length = 0
        }
    };
    esp_err_t ret = spi_device_transmit(spi_handle, (spi_transaction_t*) &transaction_ext);
    ESP_ERROR_CHECK(ret);

    // Wait until calibration conversion is rdy, maybe optimize with interrupt later, as well as in reg_read
    while (digitalRead(gpio_num_rdy)) {}
    uint16_t result = data_read();

    ESP_LOGI(TAG_ADC, "Calibration finished");
    ESP_LOGD(TAG_ADC, "Got post-calib conversion result 0x%X", result);
    return result;
}