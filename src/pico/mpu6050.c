#include "mpu6050.h"
#include "hardware/i2c.h"
#include <stdio.h>

static mpu6050_calibration_t calib = {
  .accel_offset = {0, 0, 0},
  .gyro_offset = {0, 0, 0}
};

void mpu6050_init() {
  // Reset device register to defaults
  uint8_t buf[] = {0x6B, 0x80};
  i2c_write_blocking(MPU_I2C_PORT, MPU_ADDR, buf, 2, false);
  sleep_ms(100);

  // Clear sleep mode
  buf[1] = 0x00;
  i2c_write_blocking(MPU_I2C_PORT, MPU_ADDR, buf, 2, false);
  sleep_ms(10);

  // Set sample rate to 1kHz
  buf[0] = 0x19;
  buf[1] = 0x07;
  i2c_write_blocking(MPU_I2C_PORT, MPU_ADDR, buf, 2, false);

  // Disable DLPF and external frame sync
  buf[0] = 0x1A;
  buf[1] = 0x00;
  i2c_write_blocking(MPU_I2C_PORT, MPU_ADDR, buf, 2, false);

  // Configure gyro range to +- 1000 degrees/s
  buf[0] = 0x1B;
  buf[1] = 2 << 3;
  i2c_write_blocking(MPU_I2C_PORT, MPU_ADDR, buf, 2, false);

  // Configure accellerometer range to +-8g
  buf[0] = 0x1C;
  buf[1] = 2 << 3;
  i2c_write_blocking(MPU_I2C_PORT, MPU_ADDR, buf, 2, false);
}


void mpu6050_calibrate(int num_samples) {
  float accel_sum[3] = {0};
  float gyro_sum[3] = {0};
  float raw[7];

  //printf("Calibrating MPU6050... keep device stable\n");

  for (int i = 0; i < num_samples; i++) {
    mpu6050_read_float(raw);

    // Accumulate accelerometer readings
    for (int j = 0; j < 3; j++) {
      float g_offset = 0;
      if (j == 2)
        g_offset = 1.0f;

      accel_sum[j] += raw[j] - g_offset;
    }

    // Accumulate gyroscope readings
    for (int j = 0; j < 3; j++) {
      gyro_sum[j] += raw[j + 4];
    }

    // sleep_ms(10);
  }

  // Calculate average offsets
  for (int i = 0; i < 3; i++) {
    calib.accel_offset[i] = accel_sum[i] / num_samples;
    calib.gyro_offset[i] = gyro_sum[i] / num_samples;
  }

  // printf("Calibration complete:\n");
  // printf("  Accel offsets: X=%d, Y=%d, Z=%d\n", calib.accel_offset[0],
  //        calib.accel_offset[1], calib.accel_offset[2]);
  //printf("  Gyro offsets: X=%f, Y=%f, Z=%f\n", calib.gyro_offset[0],
  //       calib.gyro_offset[1], calib.gyro_offset[2]);
}

void mpu6050_read_raw(int16_t buf[7]) {
  uint8_t start_addr = 0x3B;
  uint8_t tmp[14];

  // Retrieve data from sensor
  i2c_write_blocking(MPU_I2C_PORT, MPU_ADDR, &start_addr, 1, true);
  i2c_read_blocking(MPU_I2C_PORT, MPU_ADDR, tmp, 14, false);

  // Convert to signed shorts
  for (int i = 0; i < 7; i++) {
    buf[i] = (int16_t)(tmp[i*2] << 8 | tmp[i*2+1]);
  }

  // Apply calibration offsets
  // for (int i = 0; i < 3; i++) {
  //   buf[i] -= calib.accel_offset[i];
  //   buf[i + 4] -= calib.gyro_offset[i];
  // }
}

void mpu6050_read_float(float buf[7]) {
  int16_t tmp[7];
  
  mpu6050_read_raw(tmp);
  
  for (int i = 0; i < 3; i++) {
    buf[i] = (float)tmp[i]/4096 - calib.accel_offset[i];
    buf[i+4] = (float)tmp[i+4]/32.8 - calib.gyro_offset[i];
  }
}

// float mpu6050_read_gyro_z() {
//   uint8_t start_addr = 0x43;
//   uint8_t tmp[6];

//   i2c_write_blocking(MPU_I2C_PORT, MPU_ADDR, &start_addr, 1, true);
//   i2c_read_blocking(MPU_I2C_PORT, MPU_ADDR, tmp, 6, false);

//   uint16_t uigz = (uint16_t)(tmp[4] << 8 | tmp[5]) - calib.gyro_offset[2];

//   return uigz/32.8;
// }
