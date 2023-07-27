#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <Servo.h>

Servo servo1;
Servo servo2;

int servo1Pin = 4;  // Change to the pin connected to servo 1
int servo2Pin = 3; // Change to the pin connected to servo 2

ros::NodeHandle nh;

void jointAnglesCallback(const std_msgs::Int32MultiArray& msg) {
  // Check if the message contains the correct number of elements
  // Set servo angles
  servo1.write(msg.data[0]);
  servo2.write(msg.data[1]);
}


ros::Subscriber<std_msgs::Int32MultiArray> sub("endeffector2", &jointAnglesCallback);

void setup() {
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
