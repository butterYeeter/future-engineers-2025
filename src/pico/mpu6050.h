#ifndef MPU6050
#define MPU6050

#include <stdint.h>

// Default i2c port and pins
#define MPU_I2C_PORT i2c0
#define MPU_I2C_SDA 4
#define MPU_I2C_SCL 5
#define MPU_I2C_FREQ 400*1000

// Default address for mpu6050
#define MPU_ADDR 0x68

typedef struct {
  float accel_offset[3];
  float gyro_offset[3];
} mpu6050_calibration_t;

void mpu6050_init();

void mpu6050_calibrate(int num_samples);

void mpu6050_read_raw(int16_t [7]);

void mpu6050_read_float(float buf[7]);

// float mpu6050_read_gyro_z();

#endif /* MPU6050 */
