#ifndef QMC5883
#define QMC5883

#define QMC_I2C_PORT i2c0
#define QMC_I2C_SDA  4
#define QMC_I2C_SCL  5

typedef uint8_t QmcSettings;

typedef enum {
  STANDBY = 0,
  CONINTUOUS = 1
} QmcMode;

typedef enum {
  RATE_10 = 0 << 2,
  RATE_50 = 1 << 2,
  RATE_100 = 2 << 2,
  RATE_200 = 3 << 2
} QmcRate;

typedef enum {
  GAUSS_TWO = 0 << 4,
  GAUSS_EIGHT = 1 << 4
} QmcScale;

typedef enum {
  OSR_512 = 0 << 6,
  OSR_256 = 1 << 6,
  OSR_128 = 2 << 6,
  OSR_64 = 3 << 6
} QmcOversampleRatio;



void qmc5883_init(QmcSettings settings);

void qmc5883_raw_data(int16_t buf[3]);

#endif /* QMC5883 */
