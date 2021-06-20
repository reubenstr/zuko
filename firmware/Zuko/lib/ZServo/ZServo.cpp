#include <ZServo.h>


ZServo::ZServo()
{

}


void ZServo::Initialize(int servoPin, float minAngle, float maxAngle, float calibrationAngleOffset, float msPerDegree)
{

    _msPerDegree = msPerDegree;

    _initializedFlag = _servo.attach(servoPin, ConvertAngletoServoMs(minAngle), ConvertAngletoServoMs(maxAngle));

}

void ZServo::InitializeCalibration(int servoPin)
{

    _initializedFlag = false;

     _servo.attach(servoPin, minServoCalibrationMs, maxServoCalibrationMs);

}


void ZServo::SetAngle(float angle)
{
// TODO: TEST FOR MIN MAX ANGLE ALLOWANCE
// INTERPOLATE

    _servo.writeMicroseconds(ConvertAngletoServoMs(angle));
}

void ZServo::SetServoMs(int microseconds)
{
    _servo.writeMicroseconds(microseconds);
}


int ZServo::ConvertAngletoServoMs(float angle)
{ 
    return _msPerDegree * angle;
}
