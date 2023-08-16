#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle nh;

std_msgs::UInt8 distance_msg;
ros::Publisher pub1("ultra_sonic_1", &distance_msg);
ros::Publisher pub2("ultra_sonic_2", &distance_msg); // New publisher for the second sensor

const int trigPin1 = 10; // Trig pin for the first sensor
const int echoPin1 = 9; // Echo pin for the first sensor

const int trigPin2 = 6; // Trig pin for the second sensor
const int echoPin2 = 5; // Echo pin for the second sensor

long duration1, duration2;
int distance1, distance2;

void setup() {

  pinMode(trigPin1, OUTPUT); // Sets the trigPin1 as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin1 as an Input

  pinMode(trigPin2, OUTPUT); // Sets the trigPin2 as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin2 as an Input

  Serial.begin(57600); // Starts the serial communication

  nh.initNode();
  nh.advertise(pub1);
  nh.advertise(pub2); // Advertise the second publisher
}

void loop() {

  // First Sensor
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = duration1 * 0.034 / 2;
  distance_msg.data = int(distance1);
  pub1.publish(&distance_msg);

  // Second Sensor
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2 * 0.034 / 2;
  distance_msg.data = int(distance2);
  pub2.publish(&distance_msg);

  nh.spinOnce();
}
