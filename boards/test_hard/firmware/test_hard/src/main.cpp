#include <Arduino.h>
#include "Adafruit_NeoPixel.h"
#include "driver/spi_master.h"

#include "ADS114.hpp"

#ifdef RGB_BUILTIN
#undef RGB_BUILTIN
#endif
#define RGB_BUILTIN 38

#ifdef RGB_BRIGHTNESS
#undef RGB_BRIGHTNESS
#endif
#define RGB_BRIGHTNESS 10

#define DRDY_PIN GPIO_NUM_17

int16_t read_differencial(inpmux_muxp, inpmux_muxn, bool);
int16_t read_single(inpmux_muxp, bool);
void blink_cycle();

ADS114 ads;

const char* const TAG_MAIN = "MAIN";
double lsb_range;

void setup() {
  delay(2000);

  Serial.begin(115200);

  pinMode(GPIO_NUM_16, OUTPUT);
  pinMode(RGB_BUILTIN, OUTPUT);
  pinMode(DRDY_PIN, INPUT);
  digitalWrite(GPIO_NUM_16, HIGH);
  ads = ADS114(SPI2_HOST, DRDY_PIN);
  

  // Enable BOCS
  // ads.set_sys_mon(SYS_MON_BOCS_1);
  
  // Set input mux
  // ads.inpmux(MUXP_AIN1, MUXN_AIN0);

  // Single shot
  ads.conv_mode(DATARATE_MODE_CONTINUOUS);
  ads.start();
  // ads.set_datarate(DATARATE_MODE_CONTINUOUS, DATARATE_DR_2_5, DATARATE_CLK_INTERNAL_4096MHZ);

  int16_t ofcal_old = ads.ofcal_read();
  ESP_LOGD(TAG_MAIN, "OFCAL old value: 0d%d", ofcal_old);
  ads.calibrate();
  int16_t ofcal_new = ads.ofcal_read();
  ESP_LOGD(TAG_MAIN, "OFCAL new value: 0d%d", ofcal_new);
  ESP_LOGI(TAG_MAIN, "OFCAL delta: 0d%d", ofcal_new - ofcal_old);
  ads.conv_mode(DATARATE_MODE_SINGLE_SHOT);

  // Set PGA gain
  // ads.pga(PGA_EN_EN, GAIN128);
  ads.pga(PGA_EN_DIS, GAIN1);
  // uint8_t gain = 1;

  ads.digital_supply_voltage_read();
  ads.analog_supply_voltage_read();
  ads.set_sys_mon(SYS_MON_DIS);

  // float ref_voltage = 2.5;
  // lsb_range = (2*ref_voltage/gain)/(1<<16);
  // ESP_LOGD(TAG_MAIN, "Calculating with LSB of %fV", lsb_range);

  // Disable BOCS
  // ads.bocs(SYS_MON_DIS);

  // Set input mux
  // ads.inpmux(MUXP_AIN0, MUXN_AINCOM);

  // Single shot mode
  // ads.set_datarate(DATARATE_MODE_CONTINUOUS, DATARATE_DR_10, DATARATE_CLK_INTERNAL_4096MHZ);

  // Start conversion
  // ads.start();

  // 230uO / 0.000230
}

void loop() {

  // Disable PGA
  ads.pga(PGA_EN_DIS, GAIN1);
  uint8_t gain = 1;

  const float ref_voltage = 2.5;
  lsb_range = (2*ref_voltage/gain)/(1<<16);

  rgbLedWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,RGB_BRIGHTNESS); // White
  double abs_v = read_single(MUXP_AIN0, true) * lsb_range;
  ESP_LOGI(TAG_MAIN, "Absolute voltage: %fV", abs_v);
  Serial.print("abs:");
  Serial.print(abs_v, 8);
  rgbLedWrite(RGB_BUILTIN,0,0,0); // Turn off

  // Set PGA gain
  // ads.pga(PGA_EN_EN, GAIN128);
  ads.pga(PGA_EN_DIS, GAIN1);
  gain = 1;

  lsb_range = (2*ref_voltage/gain)/(1<<16);

  rgbLedWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,RGB_BRIGHTNESS); // White
  double diff_v = read_differencial(MUXP_AIN1, MUXN_AIN0, true) * lsb_range;
  ESP_LOGI(TAG_MAIN, "Differencial voltage: %fV", diff_v);
  Serial.print(",diff:");
  Serial.println(diff_v, 8);
  rgbLedWrite(RGB_BUILTIN,0,0,0); // Turn off
  
}


  //   ads.conv_mode(DATARATE_MODE_SINGLE_SHOT);

//   // Set PGA gain
//   ads.pga(PGA_EN_EN, GAIN2);
//   uint8_t gain = 2;

//   // Calculate lsb range
//   float ref_voltage = 2.5;
//   lsb_range = (2*ref_voltage/gain)/(1<<16);

//   rgbLedWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,RGB_BRIGHTNESS); // White
//   double abs_v = read_single(MUXP_AIN0, true) * lsb_range;
//   ESP_LOGW(TAG_MAIN, "Absolute voltage: %fV", abs_v);
//   rgbLedWrite(RGB_BUILTIN,0,0,0); // Turn off

void calibrate_res() {
  // Enable IDAC
  ads.set_idacmux(I1MUX_AIN1, I2MUX_DIS);
  ads.set_idacmag(IMAG_500u);   // Set to 500uA


  // Disable PGA
  ads.pga(PGA_EN_EN, GAIN1);
  uint8_t gain = 1;

  const float ref_voltage = 2.5;
  lsb_range = (2*ref_voltage/gain)/(1<<16);

  rgbLedWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,RGB_BRIGHTNESS); // White
  double abs_v = read_single(MUXP_AIN0, true) * lsb_range;
  ESP_LOGW(TAG_MAIN, "Absolute voltage: %fV", abs_v);
  rgbLedWrite(RGB_BUILTIN,0,0,0); // Turn off

  // Set PGA gain
  // ads.pga(PGA_EN_EN, GAIN128);
  ads.pga(PGA_EN_DIS, GAIN1);
  gain = 1;

  lsb_range = (2*ref_voltage/gain)/(1<<16);

  rgbLedWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,RGB_BRIGHTNESS); // White
  double diff_v = read_differencial(MUXP_AIN1, MUXN_AIN0, true) * lsb_range;
  ESP_LOGW(TAG_MAIN, "Differencial voltage: %fV", diff_v);
  ESP_LOGW(TAG_MAIN, "Calculated resistance in Ohm: %f", diff_v/0.0005);    // Current taken from IDAC setting
  rgbLedWrite(RGB_BUILTIN,0,0,0); // Turn off
}

int16_t read_differencial(inpmux_muxp muxp_val, inpmux_muxn muxn_val, bool single_shot) {
  // Set input mux
  ESP_LOGI(TAG_MAIN, "Reading voltage between AIN%d and AIN%d", muxp_val, muxn_val);
  ads.inpmux(muxp_val, muxn_val);

  if (single_shot) {
    ads.start();
  }

  while (digitalRead(DRDY_PIN)) {}

  return ads.data_read();
}

int16_t read_single(inpmux_muxp muxp_val, bool single_shot) {
  return read_differencial(muxp_val, MUXN_AINCOM, single_shot);
}

void blink_cycle() {
  rgbLedWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0); // Red
  delay(1000);
  rgbLedWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0); // Green
  delay(1000);
  rgbLedWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS); // Blue
  delay(1000);
  rgbLedWrite(RGB_BUILTIN,0,0,0); // Off / black
  delay(1000);
}

// Legacy code:

// // Enable IDAC
// ads.set_idacmux(I1MUX_AIN1, I2MUX_DIS);
// ads.set_idacmag(IMAG_500u);   // Set to 500uA

// ads.conv_mode(DATARATE_MODE_SINGLE_SHOT);
// rgbLedWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,RGB_BRIGHTNESS); // White
// double abs_v = read_single(MUXP_AIN0, true) * lsb_range;
// ESP_LOGW(TAG_MAIN, "Absolute voltage: %fV", abs_v);
// rgbLedWrite(RGB_BUILTIN,0,0,0); // Turn off

// // Set PGA gain
// // ads.pga(PGA_EN_EN, GAIN128);
// ads.pga(PGA_EN_EN, GAIN2);
// uint8_t gain = 2;

// float ref_voltage = 2.5;
// lsb_range = (2*ref_voltage/gain)/(1<<16);

// double diff_v = read_differencial(MUXP_AIN1, MUXN_AIN0, true) * lsb_range;
// ESP_LOGW(TAG_MAIN, "Differencial voltage: %f", diff_v);
// ESP_LOGW(TAG_MAIN, "Calculated resistance in Ohm: %f", diff_v/0.0005);    // Current taken from IDAC setting