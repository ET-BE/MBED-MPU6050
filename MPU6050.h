/**
 * MPU6050 Library for the K64F with MBED OS 6
 *
 * This code is mostly taken from:
 * https://github.com/kriswiner/MPU6050/tree/master/STM32F401
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "math.h"
#include "mbed.h"
#include "registers.h"
#include <cstdint>

#define ADO 0

#if ADO
#define MPU6050_ADDRESS 0x69 << 1 // Device address when ADO = 1
#else
#define MPU6050_ADDRESS 0x68 << 1 // Device address when ADO = 0
#endif

// Set initial input parameters
enum Ascale { AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G };

enum Gscale { GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };


/**
 * IMU class
 */
class MPU6050 {

public:

    /**
     * Constructor
     */
    MPU6050(I2C* i2c_ptr);

    int writeByte(uint8_t reg, uint8_t data);

    char readByte(uint8_t reg);

    void readBytes(uint8_t reg, uint8_t count, uint8_t *dest);

    /**
     * Read acceleartion, gryo and temperature with a single I2C read.
     *
     * This is more effecient than running both methods independently.
     */
    void readData(float* acc = nullptr, float* gyro = nullptr,
                  float* temp = nullptr);

    void readAccelData(float* destination);

    void readGyroData(float* destination);

    float readTempData();

    // Configure the motion detection control for low power accelerometer mode
    void lowPowerAccelOnly();

    void reset();

    void init();

    // Function which accumulates gyro and accelerometer data after device
    // initialization. It calculates the average of the at-rest readings and then
    // loads the resulting offsets into accelerometer and gyro bias registers.
    void calibrate();

    // Accelerometer and gyroscope self test; check calibration wrt factory
    // settings
    void selfTest(float destination[3]); // Should return percent deviation from factory trim
                                                    // values, +/- 14 or less deviation is a pass

    // Implementation of Sebastian Madgwick's "...efficient orientation filter
    // for... inertial/magnetic sensor arrays" (see
    // http://www.x-io.co.uk/category/open-source/ for examples and more details)
    // which fuses acceleration and rotation rate to produce a quaternion-based
    // estimate of relative device orientation -- which can be converted to yaw,
    // pitch, and roll. Useful for stabilizing quadcopters, etc. The performance
    // of the orientation filter is at least as good as conventional Kalman-based
    // filtering algorithms but is much less computationally intensive---it can be
    // performed on a 3.3 V Pro Mini operating at 8 MHz!
    void madgwickQuaternionUpdate(float ax, float ay, float az, float gx,
                                  float gy, float gz);

    I2C* i2c; /// Pointer to externally exisint I2C object

private:

    float getARes();

    float getGRes();

    void makeAData(const uint8_t* buffer, float* acc);

    void makeGData(const uint8_t* buffer, float* gryo);

    float makeTData(const uint8_t* buffer);

    // Specify sensor full scale
    Ascale scale_a;
    Gscale scale_g;

    // Scale resolutions per LSB for the sensors
    float aRes, gRes;
    
    // Bias corrections for gyro and accelerometer
    float gyroBias[3], accelBias[3];
    float SelfTest[6];

    // parameters for 6 DoF sensor fusion calculations
    float PI = 3.14159265358979323846f;
    float GyroMeasError = PI * (60.0f / 180.0f); // gyroscope measurement error in rads/s (start at 60
                            // deg/s), then reduce after ~10 s to 3
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
    float GyroMeasDrift = PI * (1.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick
                    // scheme usually set to a small or zero value
    
    float pitch, yaw, roll;

    float deltat = 0.0f; // integration interval for both filter schemes

    int lastUpdate = 0, firstUpdate = 0, Now = 0; // used to calculate integration interval // used to calculate
                // integration interval
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion

};

#endif
