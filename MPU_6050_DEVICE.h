#ifndef MPU_6050_DEVICE_H
#define MPU_6050_DEVICE_H
#include <Arduino.h>
#include <Wire.h>

class MPU_6050_Device {
private:
    int PIN_WIRE_SDA = 21;
    int PIN_WIRE_SCL = 22;
    int i2C_addr = 0x68;
    float temp;
    float RateRoll, RatePitch, RateYaw;
    float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
    int RateCalibrationNumber;
    float AccX, AccY, AccZ;
    float AngleRoll, AnglePitch; 
    float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
    float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
    float Kalman1DOutput[2]={0,0};
    void sensorCalibration(void);
    void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
    void calculateKalman(void);
    void readTemp(void);
    int16_t AccXLSB, AccYLSB, AccZLSB;
    int16_t GyroX, GyroY,GyroZ;
public:
    MPU_6050_Device(void); 
    MPU_6050_Device(uint8_t I2C_addr); // default I2C pins
    MPU_6050_Device(uint8_t I2C_addr, uint8_t PIN_WIRE_SDA, uint8_t PIN_WIRE_SCL); // user configured I2C pins
    void init(void);
    void update(void);
    bool connected(void);
    float getPitchAngle(void);
    float getRollAngle(void);
    float getKalmanPitchAngle(void); // Kalman filtered PitchAngle
    float getKalmanRollAngle(void);  // Kalman Filtered RollAngle
    float getGyroX(void);
    float getGyroY(void);
    float getGyroZ(void);
    float getRawGyroX(void);
    float getRawGyroY(void);
    float getRawGyroZ(void);
    float getAccX(void);
    float getAccY(void);
    float getAccZ(void);
    float getRawAccX(void);
    float getRawAccY(void);
    float getRawAccZ(void);
    float getTemperature(void);
};
#endif
