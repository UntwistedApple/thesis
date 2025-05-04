#pragma once

#include "driver/spi_master.h"
#include "regs.hpp"

enum ads_cmd {
    NOP,
    WAKEUP,
    POWERDOWN,
    RESET,
    START,
    STOP,
    SYOCAL,
    SYGCAL,
    SFOCAL,
    RDATA,
    RREG,
    WREG
};

class ADS114 {
    spi_host_device_t   spi_device;
    spi_device_handle_t spi_handle;
    gpio_num_t gpio_num_rdy;

public:
    ADS114(spi_host_device_t, gpio_num_t);
    ADS114() {};

    bool check_id();
    void start();
    uint8_t reg_read(reg_name);
    uint16_t reg_read_2(reg_name);
    void reg_write(uint8_t, uint8_t);
    void reset();
    void set_internal_reference(refcon);
    void select_reference(refsel);
    void set_ref_bufs(refp, refn);
    void pga(pga_en, pga_gain);
    void inpmux(inpmux_muxp, inpmux_muxn);
    void set_sys_mon(sys_mon);
    void set_cal_samp(cal_samp);
    void set_idacmag(imag);
    void set_idacmux(i1mux, i2mux);
    void conv_mode(datarate_mode);
    void set_datarate(datarate_mode, datarate_dr, datarate_clk);
    int16_t data_read();
    uint16_t ofcal_read();
    void register_dump();
    uint16_t calibrate();
    double digital_supply_voltage_read();
    double analog_supply_voltage_read();
};
