#include "MPU_6050_DEVICE.h"

MPU_6050_Device::MPU_6050_Device(uint8_t I2C_addr){
    this->i2C_addr = I2C_addr;
}
MPU_6050_Device::MPU_6050_Device(uint8_t I2C_addr, uint8_t PIN_WIRE_SDA, uint8_t PIN_WIRE_SCL){
    this->i2C_addr = I2C_addr;
    this->PIN_WIRE_SDA = PIN_WIRE_SDA;
    this->PIN_WIRE_SCL = PIN_WIRE_SCL;
    this->RateRoll = RateRoll;
}
//private
void MPU_6050_Device::sensorCalibration(){
    if(connected()){
        if(SERIAL_OUTPUT) Serial.println("Sensor Calibration. Don't move the device!");
        if(SERIAL_OUTPUT) Serial.println("....");
        for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
            //Serial.print(".");
    //update();
        RateCalibrationRoll+=RateRoll;
        RateCalibrationPitch+=RatePitch;
        RateCalibrationYaw+=RateYaw;
        delay(1);
        }
        RateCalibrationRoll/=2000;
        RateCalibrationPitch/=2000;
        RateCalibrationYaw/=2000;
        if(SERIAL_OUTPUT) Serial.println("Calibration Done.");
    }
}
void MPU_6050_Device::kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState=KalmanState+0.004*KalmanInput;
    KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
    KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
    KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0]=KalmanState; 
    Kalman1DOutput[1]=KalmanUncertainty;
}
void MPU_6050_Device::calculateKalman(){
    RateRoll-=RateCalibrationRoll;
    RatePitch-=RateCalibrationPitch;
    RateYaw-=RateCalibrationYaw;
    /*Roll*/
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll=Kalman1DOutput[0]; 
    KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
    /*Pitch*/
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch=Kalman1DOutput[0]; 
    KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
}

// public
void MPU_6050_Device::init(){
        // join I2C bus (I2Cdev library doesn't do this automatically)
        Wire.begin(PIN_WIRE_SDA, PIN_WIRE_SCL);
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        delay(250);
        if(connected()){
            if(SERIAL_OUTPUT) Serial.println("\nInitializing MPU.");
            Wire.beginTransmission(i2C_addr); 
            Wire.write(i2C_addr); // Power management
            Wire.write(0x6B);
            Wire.write(0x00); // start Sensors
            Wire.endTransmission();
            Wire.beginTransmission(i2C_addr); // lowpass filter - Acc and Gyro
            Wire.write(0x1A); // Gyro >> >> 5 - 10hz ; 4 - 20Hz ; 3 - 42Hz ; 2 - 98Hz ; 1 - 188Hz ; 0 - 256Hz; 
            Wire.write(0x05); //Acc >> 5 - 10hz ; 4 - 21Hz ; 3 - 44Hz ; 2 - 94Hz ; 1 - 184Hz ; 0 - 260Hz; 
            Wire.endTransmission();
            Wire.beginTransmission(i2C_addr); 
            Wire.write(0x1B); // Gyro sensitivity scale factor
            Wire.write(0x8); // 0 - 250deg/sec ; 1 - 500deg/sec ; 2 - 1000deg/sec ; 3 - 2000deg/sec ;
            Wire.endTransmission();
            Wire.beginTransmission(i2C_addr);
            Wire.write(0x1C); // Gyro 
            Wire.write(0x10);
            Wire.endTransmission();
            sensorCalibration();
            if(SERIAL_OUTPUT) Serial.println("MPU6050 init Done.");
    }
}

bool MPU_6050_Device::connected(){
    Wire.beginTransmission(i2C_addr);
    if (Wire.endTransmission () == 0){
         return true;
         if(SERIAL_OUTPUT) Serial.println("\nMPU6050 found @ 0X" + String(i2C_addr,HEX));
         FOUND_MPU = true;
         ERROR_SEND = false;
     } // end of good response
     else {
         if(!ERROR_SEND){
                if(SERIAL_OUTPUT) Serial.println("\nMPU6050 not found @ 0X" + String(i2C_addr,HEX));
                ERROR_SEND = true;
         }
         return false;
     }
}
void MPU_6050_Device::update(){
    if(connected()){
        Wire.beginTransmission(i2C_addr);
        Wire.write(0x3B); // get gyro data
        Wire.endTransmission(); 
        Wire.requestFrom(i2C_addr,6);
        AccXLSB = Wire.read() << 8 | Wire.read();
        AccYLSB = Wire.read() << 8 | Wire.read();
        AccZLSB = Wire.read() << 8 | Wire.read();
        Wire.beginTransmission(i2C_addr);
        Wire.write(0x43); // get acc data
        Wire.endTransmission();
        Wire.requestFrom(i2C_addr,6);
        GyroX=Wire.read()<<8 | Wire.read();
        GyroY=Wire.read()<<8 | Wire.read();
        GyroZ=Wire.read()<<8 | Wire.read();
        RateRoll=(float)GyroX/65.5;
        RatePitch=(float)GyroY/65.5;
        RateYaw=(float)GyroZ/65.5;
        AccX=(float)AccXLSB/4096;
        AccY=(float)AccYLSB/4096;
        AccZ=(float)AccZLSB/4096;
        AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
        AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
        calculateKalman();
        readTemp();
    }
    else {
        AngleRoll = NULL;
        AnglePitch = NULL;
        KalmanAngleRoll = NULL;
        KalmanAnglePitch = NULL;
        temp = NULL;
    }
}
float MPU_6050_Device::getTemperature(){
        return temp;
}
void MPU_6050_Device::readTemp(){ // read MPU6050 temp.
    Wire.beginTransmission(i2C_addr);
    Wire.write(0x41); // 41 Temp
    Wire.endTransmission(); 
    Wire.requestFrom(0x68,2);
    temp = Wire.read() << 8 | Wire.read();
    temp = (temp /340.) + 34.53; // 18.53; // + 521) + (35 * 340)) / 340;  // Simplified: chip_temperature = (value_read + 12421) / 340
}
float MPU_6050_Device::getPitchAngle(){
    return AnglePitch;
}
float MPU_6050_Device::getRollAngle(){
    return AngleRoll;
}
float MPU_6050_Device::getKalmanPitchAngle(){
    return KalmanAnglePitch;
}
float MPU_6050_Device::getKalmanRollAngle(){
    return KalmanAngleRoll;
}
float MPU_6050_Device::getGyroX(){
    return RatePitch;
}
float MPU_6050_Device::getGyroY(){
    return RateRoll;
}
float MPU_6050_Device::getGyroZ(){
    return RateYaw;
}
float MPU_6050_Device::getRawGyroX(){
    return GyroX;
}
float MPU_6050_Device::getRawGyroY(){
    return GyroY;
}
float MPU_6050_Device::getRawGyroZ(){
    return GyroZ;
}
float MPU_6050_Device::getAccX(){
    return AccX;
}
float MPU_6050_Device::getAccY(){
    return AccX;
}
float MPU_6050_Device::getAccZ(){
    return AccZ;
}
float MPU_6050_Device::getRawAccX(){
    return AccXLSB;
}
float MPU_6050_Device::getRawAccY(){
    return AccYLSB;
}
float MPU_6050_Device::getRawAccZ(){
    return AccZLSB;
}

void MPU_6050_Device::enableSerialOutput(bool serOut){
        if(serOut){
            SERIAL_OUTPUT = true;
        } else{
            SERIAL_OUTPUT = false;
        }
}




