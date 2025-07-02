#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
// #include "qmc5883.h"
#include "math.h"

#define GET_ANGLE 'a'
#define CALIBRATE 'c'
#define RESET_ANGLE 'r'

float bi[] = {-43.924241, 12.448210, -73.133957};
float A[3][3] = {{1.096529, -0.033064, -0.055229},
                {-0.033064, 0.989653, -0.120042},
                {-0.055229, -0.120042, 0.819875}};

void apply_calib(float buf[3]) {
    float tmp[3] = {buf[0] - bi[0], buf[1] - bi[1], buf[2] - bi[2]};

    for (int i = 0; i < 3; i++) {
        buf[i] = A[i][0] * tmp[0] + A[i][1] * tmp[1] + A[i][2] * tmp[2];
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
    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);

    gpio_put(25, true);
    mpu6050_init();
    sleep_ms(3000);

    gpio_put(25, false);
    mpu6050_calibrate(10000);
    gpio_put(25, true);
    // qmc5883_init(CONINTUOUS | RATE_200 | GAUSS_EIGHT | OSR_512);


    uint64_t last_time, current_time;
    current_time = to_us_since_boot(get_absolute_time());
    last_time = current_time;
    uint64_t last_print = current_time;

    float angle = 0.0f;         // Integrated angle using gyro
    float gyro_prev = 0.0f;     // Previous value for apply low pass filter
    float RC = 0.000265f;       // Filter out noise above ~600HZ
    while (true) {
        float buf[7];
        mpu6050_read_float(buf);

        // Calculate delta time between loops
        current_time = to_us_since_boot(get_absolute_time());
        float delta_time = (float)(current_time - last_time)/1000000;
        last_time = current_time;
        
        
        // int16_t raw[3];
        // qmc5883_raw_data(raw);
        // float b[3] = {raw[0]*0.033333f, raw[1] * 0.033333f, raw[2] * 0.033333f};
        // apply_calib(b);

        // Low pass filter
        float alpha = delta_time / (RC + delta_time);
        gyro_prev = alpha * buf[6] + (1-alpha) * gyro_prev;
        angle += delta_time * gyro_prev;

        // Non blocking input check
        int c = getchar_timeout_us(0);

        // Send current angle if ev3 requests it
        switch (c) {
            case GET_ANGLE:
                stdio_put_string((char*)&angle, sizeof(float), false, false);
                bool state = gpio_get(25);
                gpio_put(25, state ^ true);
                break;
            case CALIBRATE:
                char num_samples = getchar_timeout_us(100);
                mpu6050_calibrate(num_samples * 100);
                break;
            case RESET_ANGLE:
                angle = 0.0f;
                break;
        }

        if (current_time - last_print > 500000) {
            bool state = gpio_get(15);
            gpio_put(15, state ^ true);
            last_print = current_time;
        }
    }
}
