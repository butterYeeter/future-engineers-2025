#include "hardware/i2c.h"
#include "qmc5883.h"

#define QMC_ADDR  0x0d
#define DATA  0x00
#define CTRL1 0x09
#define CTRL2 0x0a
#define FBR   0x0b


typedef enum {
  INTERRUPT_DISABLE = 1,
  POINTER_ROLL_OVER = 1 << 6
} QmcControl;



void qmc5883_init(QmcSettings settings) {
  uint8_t cmd[2] = {CTRL1, settings};
  i2c_write_blocking(QMC_I2C_PORT, QMC_ADDR, cmd, 2, true);

  cmd[0] = CTRL2;
  cmd[1] = INTERRUPT_DISABLE | POINTER_ROLL_OVER;
  i2c_write_blocking(QMC_I2C_PORT, QMC_ADDR, cmd, 2, true);

  cmd[0] = FBR;
  cmd[1] = 0x01;
  i2c_write_blocking(QMC_I2C_PORT, QMC_ADDR, cmd, 2, false);
}

// void qmc5883_raw_data(int16_t buf[3]) {
//   // uint8_t tmp[6];
//   uint8_t reg = DATA;
//   i2c_write_blocking(QMC_I2C_PORT, QMC_ADDR, &reg, 1, false);
//   i2c_read_blocking(QMC_I2C_PORT, QMC_ADDR, (uint8_t*)buf, 6, false);

//   // for (int i = 0; i < 3; i++) {
//   //   buf[i] = (int16_t)
//   // }
// }

void qmc5883_raw_data(int16_t buf[3]) {
  uint8_t tmp[6];
  uint8_t reg = DATA;
  i2c_write_blocking(QMC_I2C_PORT, QMC_ADDR, &reg, 1, false);
  i2c_read_blocking(QMC_I2C_PORT, QMC_ADDR, tmp, 6, false);

  for (int i = 0; i < 3; i++) {
    buf[i] = (int16_t)(tmp[2 * i] | (tmp[2 * i + 1] << 8));
  }
}