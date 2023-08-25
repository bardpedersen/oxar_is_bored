// For older arduino IDEs, use WProgram.h instead of Arduino.h
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

// Include ros libraries, the message type and the servo library
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <Servo.h>

// Create servo objects
Servo servo1;
Servo servo2;

int servo1Pan = 3; // Pin connected to servo 1, (pan)
int servo2Tilt = 4; // Pin connected to servo 2, (tilt)

// Create a node handle
ros::NodeHandle nh;

// Callback function for the subscriber
void jointAnglesCallback(const std_msgs::Int32MultiArray& msg) {
  // Set servo angles from corresponding values in the message

  servo1.write(msg.data[0]+8); //needed for 90 to be straight
  servo2.write(msg.data[1]); 
}

// Create a subscriber object
ros::Subscriber<std_msgs::Int32MultiArray> sub("endeffector2", &jointAnglesCallback);

// Setup function
void setup() {
  // Initialize the serial port and set the baud rate (need to be same as in launch file)
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);

  // Initialize the node handle and subscriber
  nh.initNode();
  nh.subscribe(sub);

  // Attach the servos to the pins
  servo1.attach(servo1Pan);
  servo2.attach(servo2Tilt);
}

// Loop function
void loop() {
  nh.spinOnce();
  delay(1);
}
