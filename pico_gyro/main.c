#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "qmc5883.h"

void input_callback(void *ptr) {

    char c = stdio_getchar_timeout_us(0);

    if(c == 'd') {
        bool *send_data = (bool*)ptr;
        *send_data = true;
        gpio_put(25, false);
    }
}

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

    // gpio_put(25, true);
    // mpu6050_init();
    // sleep_ms(1000);

    // gpio_put(25, false);
    // mpu6050_calibrate(300);
    // gpio_put(25, true);
    qmc5883_init(CONINTUOUS | RATE_200 | GAUSS_EIGHT | OSR_512);


    uint64_t last_time, current_time;
    current_time = to_us_since_boot(get_absolute_time());
    last_time = current_time;

    float angle = 0.0f;
    bool send_data = false;

    // stdio_set_chars_available_callback(input_callback, &send_data);

    while (true) {
        // float buf[7];

        // mpu6050_read_float(buf);

        int16_t buf[3];
        qmc5883_raw_data(buf);

        current_time = to_us_since_boot(get_absolute_time());
        float delta_time = (float)(current_time - last_time)/1000000;
        last_time = current_time;


        float b[3] = {buf[0]*0.033333f, buf[1] * 0.033333f, buf[2] * 0.033333f};
        stdio_put_string((char*)b, 12, false, false);

        // printf("%f,%f,%f\n", buf[0]*0.033333f, buf[1] * 0.033333f, buf[2] * 0.033333f);
        


        // sleep_ms(50);
        // printf("TEST\n");
        // angle += delta_time * buf[6];

        // if (send_data) {
        //     stdio_put_string((char*)&angle, 4, false, false);
        //     send_data = false;
        // }
    }
}
