#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <Servo.h>

Servo servo1;
Servo servo2;

int servo1Pin = 3;  // Change to the pin connected to servo 1
int servo2Pin = 4; // Change to the pin connected to servo 2

ros::NodeHandle nh;
std_msgs::Int32MultiArray jointAngles;

void jointAnglesCallback(const std_msgs::Int32MultiArray& msg) {
  // Check if the message contains the correct number of elements
  if (msg.data_length == 2) {
    // Extract servo angles from the message
    int angle1 = msg.data[0];
    int angle2 = msg.data[1];

    // Set servo angles
    servo1.write(angle1);
    servo2.write(angle2);
  }
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
}
