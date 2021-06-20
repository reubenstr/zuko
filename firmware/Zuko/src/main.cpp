#include <Arduino.h>

#include <ZServo.h>
#include <ros.h>
#include <std_msgs/Empty.h>

#include "std_msgs/String.h"
#include <std_msgs/Float32.h>

#include <main.h>

ros::NodeHandle rosNodeHandle;

ZServo frontLeftShoulder, frontLeftWrist;

void messageCb(const std_msgs::Empty &toggle_msg)
{
  digitalWrite(LED_BUILTIN, ~digitalRead(LED_BUILTIN)); // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);

void InitServos()
{

  frontLeftShoulder.Initialize(frontLeftShoulderServoPin, -90, 90, 0, 5.5);
  frontLeftWrist.Initialize(frontLeftWristServoPin, -90, 90, 0, 5.5);
}

void blinkLED()
{

  static unsigned long start = millis();
  static bool toggle = true;
  unsigned long delay = toggle ? 1000 : 100;
  if (millis() - start > delay)
  {
    start = millis();
    toggle = !toggle;
    digitalWrite(LED_BUILTIN, toggle);
  }
}

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);

  rosNodeHandle.initNode();
  rosNodeHandle.subscribe(sub);
  rosNodeHandle.advertise(pub_temp);

  //InitServos();

  frontLeftShoulder.InitializeCalibration(frontLeftShoulderServoPin);
  frontLeftWrist.InitializeCalibration(frontLeftWristServoPin);

  frontLeftShoulder.SetServoMs(1500);
  frontLeftWrist.SetServoMs(1500);

  // TEMP TEMP TEM PSTOP HJERE
  //while (1)
  {
  }
}

void loop()
{

  blinkLED();

  static unsigned long start = millis();
  if (millis() - start > 1000)
  {
    start = millis();
  }

 frontLeftShoulder.SetServoMs(600);
  frontLeftWrist.SetServoMs(600);
  delay(5000);
  //frontLeftShoulder.SetServoMs(1500);
  //delay(2000);
   frontLeftShoulder.SetServoMs(1500);
  frontLeftWrist.SetServoMs(1500);
  delay(5000);
  //frontLeftShoulder.SetServoMs(1500);
  //delay(2000);

  /*
  rosNodeHandle.spinOnce();


  static int count;
  count++;
  temp_msg.data = count;
  pub_temp.publish(&temp_msg);
  */
}