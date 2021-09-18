#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <quad_interfaces/msg/motion_servos.h>

#include <main.h>
#include <zservo.h> // Local library.

// TEMP
#include <Servo.h> // Arduinoteensy framework's servo library.



// micro_ros variables
quad_interfaces__msg__MotionServos motionServos;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// quad variables
const int numMotionServos = 12;
ZServo *zservos[numMotionServos];

// pin map from teensy pins to PCB pins (index is servo, value is pin).
const int motionServosPinMap[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

#define LED_PIN 13

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
     digitalWrite(LED_PIN, LOW);
    delay(1000);
  }
}

void subscription_callback(const void *msgin)
{
  const quad_interfaces__msg__MotionServos *motionServosMsg = (const quad_interfaces__msg__MotionServos *)msgin;

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  for (int i = 0; i < numMotionServos; i++)
  {   
    zservos[i]->SetServoMs(motionServosMsg->pulse_width[i]);
    zservos[i]->SetEnable(motionServosMsg->enable[i]);
  }
}

void setupMicroros()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(quad_interfaces, msg, MotionServos),
      "motion_servos"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &motionServos, &subscription_callback, ON_NEW_DATA));
}

void setup()
{
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  delay(1000);

  setupMicroros();

  // Setup servos
  for (int i = 0; i < numMotionServos; i++)
  {
    zservos[i] = new ZServo(motionServosPinMap[i]);    
  }
}

void loop()
{  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
