#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu6050.h"


int main()
{
    stdio_init_all();

    i2c_init(MPU_I2C_PORT, 400000);
    
    gpio_set_function(MPU_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(MPU_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(MPU_I2C_SDA);
    gpio_pull_up(MPU_I2C_SCL);

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    gpio_put(25, true);
    mpu6050_init();
    sleep_ms(1000);

    gpio_put(25, false);
    mpu6050_calibrate(300);
    gpio_put(25, true);


    uint64_t last_time, current_time;
    current_time = to_us_since_boot(get_absolute_time());
    last_time = current_time;
    while (true) {
        float buf[7];

        mpu6050_read_float(buf);

        current_time = to_us_since_boot(get_absolute_time());
        uint64_t delta_time = current_time - last_time;
        last_time = current_time;

        // *(uint64_t*)(buf+5) = delta_time;
        // stdio_put_string((char*)(buf+4), 12, false, false);
        printf("%10f, %lu\n", buf[6], delta_time);
        // sleep_ms(1000);
    }
}
