# MBED-MPU6050

Library for the MPU6050 accelerometer/gyroscope. Written for the K64F running MBED OS 6.

## Example

Use it like:

```c++
#include "mbed.h"
#include "mbed-mpu6050/MPU6050.h"

I2C i2c(I2C_SDA, I2C_SCL);
MPU6050 imu(&i2c);


int main()
{
    printf("Starting...\n\r");

    i2c.frequency(400000); // 400 kHz

    uint8_t whoami = imu.readByte(WHO_AM_I_MPU6050);

    if (whoami != 0x68) {
        printf("Invalid device...\n\r");
        return 1; // Correct device was not found
    }

    imu.reset(); // Reset registers to default in preparation for device calibration
    imu.calibrate(); // Calibrate gyro and accelerometers, load biases in bias registers
    imu.init();

    float a[3], g[3];

    while (true) {

        // If data ready bit set, all data registers have new data
        if (imu.readByte(INT_STATUS) & 0x01)    // check if data ready interrupt
        {
            imu.readAccelData(a);  // Read the x/y/z acceleration
    
            printf("ax: %d, ay: %d, az: %d\n\r", (int)(a[0] * 1000.0f), (int)(a[1] * 1000.0f), (int)(a[2] * 1000.0f));
    
            imu.readGyroData(g);  // Read the x/y/z gyroscope values       

            printf("gx: %d, gy: %d, gz: %d\n\r", (int)(g[0] * 1000.0f), (int)(g[1] * 1000.0f), (int)(g[2] * 1000.0f));     

            float temp = imu.readTempData();

            printf("Temp: %d\n\r", (int)(temp * 1000.0f));     
        }

        thread_sleep_for(100); // 10 Hz
    }
}

```
