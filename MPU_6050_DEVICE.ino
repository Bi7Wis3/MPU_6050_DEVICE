/*
        ---------- MPU6050 libarary V1.0 ----------
              By B1TwIs3 Op3r4ToR - no(C)2023

              Todo:
              - Add quarternations and other calculations
              - 
*/
#include "MPU_6050_DEVICE.h"

uint8_t i2c_address = 0x68;
uint8_t sda_pin = 18;
uint8_t scl_pin = 17;

float PitchAngleCorrection = -0.0; // user correction. to zero out angles.
float RollAngleCorrection = -0.0;  // user correction. to zero out angles.

//MPU_6050_Device mpu(i2c_address); // for custom i2c address and default I2C pins - 0x69 for devices with A0 set to HIGH.
MPU_6050_Device mpu(i2c_address, sda_pin, scl_pin ); // for custom i2c and custom I2C pins

void setup() {
    Serial.begin(115200);
    mpu.enableSerialOutput(true); 
    mpu.init();
}

void loop() {
    mpu.update();
    if(mpu.connected()){
        Serial.print(String(mpu.getKalmanPitchAngle() - PitchAngleCorrection ) + ", " );
        Serial.print(String(mpu.getKalmanRollAngle() - RollAngleCorrection ) + ", ");
        Serial.print(String(mpu.getPitchAngle() ) + ", " );
        Serial.print(String(mpu.getRollAngle() ) + ", ");
        Serial.print(String(mpu.getGyroX() ) +", " );
        Serial.print(String(mpu.getGyroY() ) +", " );
        Serial.print(String(mpu.getGyroZ() ) +", " );
        Serial.print(String(mpu.getRawGyroX() ) +", " );
        Serial.print(String(mpu.getRawGyroY() ) +", " );
        Serial.print(String(mpu.getRawGyroZ() ) +", " );
        Serial.print(String(mpu.getAccX() ) +", " );
        Serial.print(String(mpu.getAccY() ) +", " );
        Serial.print(String(mpu.getAccZ() ) +", " );
        Serial.print(String(mpu.getRawAccX() ) +", " );
        Serial.print(String(mpu.getRawAccY() ) +", " );
        Serial.print(String(mpu.getRawAccZ() ) +", " );
        Serial.print(String(mpu.getTemperature(),1) );
        Serial.println();
    }
}
