#include <zservo.h>

ZServo::ZServo(int servoPin)
{
    _servoPin = servoPin;
}

/*
void ZServo::Initialize(int servoPin, float minAngle, float maxAngle, float calibrationOffsetAngle, float msPerDegree)
{
    _servoPin = servoPin;
    _minAngleDegrees = minAngle;
    _maxAngleDegrees = maxAngle;
    _calibrationOffsetAngle = calibrationOffsetAngle;
    _msPerDegree = msPerDegree;

    Enable();
}
*/

/*
void ZServo::SetAngleInRadians(float angleRadians)
{   
    SetAngleInDegrees(angleRadians * (180.0 / 3.1415));
}

void ZServo::SetAngleInDegrees(float angleDegrees)
{
    _angleDegrees = angleDegrees;

    if (_angleDegrees < _minAngleDegrees)
        _angleDegrees = _minAngleDegrees;
    if (_angleDegrees > _maxAngleDegrees)
        _angleDegrees = _maxAngleDegrees;

    _servo.writeMicroseconds(ConvertAngletoServoMs(_angleDegrees));
}
*/



void ZServo::SetServoMs(int pulseWidth)
{
    _pulseWidth = pulseWidth;
    _servo.writeMicroseconds(_pulseWidth);
}

void ZServo::Enable()
{
    _enabled = true;
    //_initialized = _servo.attach(_servoPin, ConvertAngletoServoMs(_minAngle), ConvertAngletoServoMs(_maxAngle));
    _initialized = _servo.attach(_servoPin);
}

void ZServo::Disable()
{
    _enabled = false;
    _servo.detach();
}

void ZServo::SetEnable(bool enableFlag)
{
    if (enableFlag && !_servo.attached())
    {
        Enable();
        //SetAngleInDegrees(_angleDegrees);
        SetServoMs(_pulseWidth);
    }
    else if (!enableFlag && _servo.attached())
    {
        Disable();
    }
}

/*
int ZServo::ConvertAngletoServoMs(float angle)
{
    return _msPerDegree * angle + _servoCenteredMs;
}
*/