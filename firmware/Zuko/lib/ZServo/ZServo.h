
#ifndef ZSERVO_INCLUDE_GUARD
#define ZSERVO_INCLUDE_GUARD

// Use the arduinoteensy framework's servo library
#include <Servo.h>

#include <Arduino.h>
#include <math.h>






class ZServo
{

    public:

    // Default contructor.
    ZServo();

    void Initialize(int servoPin, float minAngle, float maxAngle, float calibrationAngleOffset, float msPerDegree);
     void InitializeCalibration(int servoPin);


    void SetAngle(float angle);
    void SetServoMs(int microseconds);

    private:

    int ConvertAngletoServoMs(float angle);

    Servo _servo;
    bool _initializedFlag = false;
    float _msPerDegree;

    const int minServoCalibrationMs = 500;
    const int maxServoCalibrationMs = 2500;

};

#endif