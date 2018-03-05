#include <Wire.h>
#include "Mpu9250.h"

Mpu9250::Mpu9250()
{
  connected = false;
  yaw_rate = 0;
}

Mpu9250::~Mpu9250()
{
  //
}

void Mpu9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t Mpu9250::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                            // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void Mpu9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
    dest[i++] = Wire.read();         // Put read results in the Rx buffer
  }
}

void Mpu9250::readAccelData(int16_t * destination) {
  uint8_t rawData[6];
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void Mpu9250::readGyroData(int16_t * destination) {
  uint8_t rawData[6];
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void Mpu9250::getGres() {
  switch (Gscale)
  {
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void Mpu9250::getAres() {
  switch (Ascale)
  {
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

void Mpu9250::processMpu9250Data() {
  // Now we'll calculate the accleration value into actual g's
  ax = (float)accelValue[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
  ay = (float)accelValue[1]*aRes; // - accelBias[1];   
  az = (float)accelValue[2]*aRes; // - accelBias[2];
    // Calculate the gyro value into actual degrees per second
  gx = (float)gyroValue[0]*gRes;  // get actual gyro value, this depends on scale being set
  gy = (float)gyroValue[1]*gRes;  
  gz = (float)gyroValue[2]*gRes;
  
  yaw_rate = int(gz);
}

void Mpu9250::readMpu9250Raw() {
  readAccelData(accelValue);
  readGyroData(gyroValue);
}

void Mpu9250::readMpu9250Data() {
  if (connected) {
    static unsigned long lastReading;
    if (micros() - lastReading >= READ_FREQUENCY) {           // Read the data every 8000us (equals 125Hz)
      lastReading = micros();
  
      readMpu9250Raw();                                                  // Read RAW data
      processMpu9250Data();                                              // Process the MPU 6050 data
    }
  }
}

void Mpu9250::isOnline() {
  uint8_t c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  if (c == 0x71) {
    if (Serial) {
      Serial.println("MPU9250 is online...");
    }
    connected = true;
  } else {
    if (Serial) {
      Serial.println("MPU9250 is not connected...");
    }
    connected = false;
  }
}

void Mpu9250::initMPU9250() {
  // Activate device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, PWR_ON);                 // Activate the MPU-9250
  delay(100);                                                   // Wait for all registers to reset 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, Ascale);             // Set accelerometer full-scale to 2 g, maximum sensitivity
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, Gscale);              // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  getAres();                                                    // Set DPS/ACD tick depending of sensibility
  getGres();
}

void Mpu9250::setupMpu9250(uint8_t sAscale, uint8_t sGscale) {
  Wire.begin();
  Ascale = sAscale;
  Gscale = sGscale;
  // Is device connected
  isOnline();
  if (connected) {
    initMPU9250();
  } else {
    yaw_rate = 0;
  }
}