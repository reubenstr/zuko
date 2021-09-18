#ifndef ZSERVO_INCLUDE_GUARD
#define ZSERVO_INCLUDE_GUARD

#include <Arduino.h>
#include <Servo.h> // Arduinoteensy framework's servo library.
#include <math.h>

class ZServo
{
public:
    // Default contructor.
    ZServo(int servoPin);

    //void Initialize(int servoPin, float minAngle, float maxAngle, float calibrationAngleOffset, float msPerDegree);

    //void SetAngleInRadians(float angleRadians);
    //void SetAngleInDegrees(float angleDegrees);
    void SetServoMs(int microseconds);

    void Enable();
    void Disable();
    void SetEnable(bool enableFlag);

private:
    //int ConvertAngletoServoMs(float angle);

    Servo _servo;
    bool _initialized = false;
    bool _enabled = false;

    int _servoPin;
    float _angleDegrees;
    int _pulseWidth;
    float _minAngleDegrees = -90.0;
    float _maxAngleDegrees = 90.0;
    float _calibrationOffsetAngle = 0.0;
    float _msPerDegree = 11.11;

    //const int _minServoCalibrationMs = 500;
    //const int _maxServoCalibrationMs = 2500;
    const int _servoCenteredMs = 1500;
};

#endif