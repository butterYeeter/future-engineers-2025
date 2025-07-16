#ifndef TCS34725
#define TCS34725

#include "hardware/i2c.h"

#define TCS_INT 6
#define TCS_SDA 4
#define TCS_SCL 5
#define TCS_PORT i2c0

#define TCS34725_ADDRESS 0x29
#define TCS34725_COMMAND_BIT 0x80

#define TCS34725_ENABLE 0x00
#define TCS34725_ATIME 0x01
#define TCS34725_CDATAL 0x14
#define TCS34725_RDATAL 0x16
#define TCS34725_GDATAL 0x18
#define TCS34725_BDATAL 0x1A
#define TCS34725_ID 0x12
#define TCS34725_AILTL 0x04
#define TCS34725_AILTH 0x05
#define TCS34725_AIHTL 0x06
#define TCS34725_AIHTH 0x07
#define TCS34725_PERS 0x0C

#define TCS34725_ENABLE_PON 0x01
#define TCS34725_ENABLE_AEN 0x02
#define TCS34725_ENABLE_AIEN 0x10

bool tcs_init();

void tcs_get_rgb(float buf[3]);

void tcs_get_raw(uint16_t buf[4]);

bool tcs_data_available();

#endif /* TCS34725 */
