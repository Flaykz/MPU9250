#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"

#define MPU9250_ADDRESS   0x68 // Address of MPU9250
#define WHO_AM_I_MPU9250  0x75 // Should return 0x71
#define PWR_MGMT_1        0x6B // Device defaults to the SLEEP mode
#define PWR_ON            0x00 // Activate device
#define ACCEL_CONFIG      0x1C
#define GYRO_CONFIG       0x1B
#define CONFIG            0x1A
#define ACCEL_XOUT_H      0x3B
#define GYRO_XOUT_H       0x43

#define AFS_2G            0x00
#define AFS_4G            0x01
#define AFS_8G            0x10
#define AFS_16G           0x11
#define GFS_250DPS        0x00
#define GFS_500DPS        0x01
#define GFS_1000DPS       0x10
#define GFS_2000DPS       0x11

#define READ_FREQUENCY    8000

class Mpu9250
{
  private:
    //
  public:
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    uint8_t readByte(uint8_t address, uint8_t subAddress);
    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
    void readAccelData(int16_t * destination);
    void readGyroData(int16_t * destination);
    void getGres();
    void getAres();
    void processMpu9250Data();
    void readMpu9250Raw();
    void readMpu9250Data();
    void isOnline();
    void initMPU9250();
    void setupMpu9250(uint8_t sAscale, uint8_t sGscale);
    float ax, ay, az, gx, gy, gz;   // Stores final value of accel and gyro 3 axes
    float yaw_rate;
  protected:
    bool connected;
    uint8_t Ascale;                 // Accelerometer résolution
    uint8_t Gscale;                 // Gyroscope résolution
    float aRes, gRes;               // scale resolutions per LSB for the sensors
    int16_t accelValue[3];          // Stores the 16-bit signed accelerometer sensor output
    int16_t gyroValue[3];           // Stores the 16-bit signed gyro sensor output
}

#endif