#include "tcs34725.h"

#include "pico/stdlib.h"
#include "hardware/irq.h"

uint16_t r_val = 0;
uint16_t g_val = 0;
uint16_t b_val = 0;
uint16_t c_val = 0;

void tcs_write8(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {TCS34725_COMMAND_BIT | reg, value};
  i2c_write_blocking(i2c0, TCS34725_ADDRESS, buffer, 2, false);
}

uint8_t tcs_read8(uint8_t reg) {
  uint8_t cmd = TCS34725_COMMAND_BIT | reg;
  uint8_t value;
  i2c_write_blocking(i2c0, TCS34725_ADDRESS, &cmd, 1, true);
  i2c_read_blocking(i2c0, TCS34725_ADDRESS, &value, 1, false);
  return value;
}

uint16_t tcs_read16(uint8_t reg) {
  uint8_t cmd = TCS34725_COMMAND_BIT | reg;
  uint8_t buffer[2];
  i2c_write_blocking(i2c0, TCS34725_ADDRESS, &cmd, 1, true);
  i2c_read_blocking(i2c0, TCS34725_ADDRESS, buffer, 2, false);
  return (buffer[1] << 8) | buffer[0];
}

void tcs_clear_interrupt() {
  uint8_t cmd = 0x66 | 0xE0;
  i2c_write_blocking(i2c0, TCS34725_ADDRESS, &cmd, 1, false);
}

void gpio_callback(uint gpio, uint32_t events) {
  if (gpio == TCS_INT && (events & GPIO_IRQ_EDGE_FALL)) {
    c_val = tcs_read16(TCS34725_CDATAL);
    r_val = tcs_read16(TCS34725_RDATAL);
    g_val = tcs_read16(TCS34725_GDATAL);
    b_val = tcs_read16(TCS34725_BDATAL);
    tcs_clear_interrupt();
  }
}

bool tcs_init() {
  // Fastest integration time: 2.4ms
  tcs_write8(TCS34725_ATIME, 0xFF);

  tcs_write8(TCS34725_PERS, 0x00);

  tcs_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  sleep_ms(3);
  tcs_write8(TCS34725_ENABLE,
             TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN | TCS34725_ENABLE_AIEN);
  gpio_set_irq_enabled_with_callback(TCS_INT, GPIO_IRQ_EDGE_FALL, true,
                                     &gpio_callback);

  return true;
}

void tcs_get_rgb(float buf[3]) {
  if (c_val == 0) {
    *(buf) + *(buf+1) + *(buf+2) = 0.0f;
    return;
  }
  *(buf) = (float)r_val / c_val;
  *(buf+1) = (float)g_val / c_val;
  *(buf+2) = (float)b_val / c_val;
}