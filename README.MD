        ---------- Fast MPU6050 libarary V1.0 ----------
                    Pitch & Roll Output.
                     with Kalman Filter
              By B1TwIs3 0p3r4ToR - no(C)2023
   With thanks to : https://github.com/CarbonAeronautics for inspiration
        Tested on Arduino Pro mini, ESP32, Not working on ESP32-S3.

Notes: 
              Todo:
              - Add quarternations and other calculations
              - 

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
    void enableSerialOutput(bool serOut);
