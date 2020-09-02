#include <Arduino.h>
#include <NewPing.h>
#include <ros.h>
#include <std_msgs/String.h>

// ARDUINO ULTRASONIC RANGE PUBLISHER
// This script will allow users to publish ultrasonic sensor data to ROS
// via Arduino formatted as sensor_msgs/Range message

#define PI 3.1416
#define MIN_DISTANCE 0
#define MAX_DISTANCE 200 // Max distance for obstacle detection
#define FIELD_OF_VIEW 0.26
#define PING_INTERVAL 50 // Milliseconds between two consequent readings

// Arduino Pins
// Front sonar
#define TRIGGER_SONAR 8
#define ECHO_SONAR 9


// Topic names
#define TOPIC_SONAR "/sonar_raw"

// Register the ultrasonic sensors as instances of the class NewPing 
// that will hide the low level logic to extract sensor readings
NewPing sonar = NewPing(TRIGGER_SONAR, ECHO_SONAR, MAX_DISTANCE);

// Init the ROS node
ros::NodeHandle nh;

// Message that will be created and published on ros
std_msgs::String sonar_msg;
 
// Topics on which the sonar messages will be published
ros::Publisher pub_sonar(TOPIC_SONAR, &sonar_msg);
 
// Raw data coming from sensors
uint8_t sonarDist;

char frameid[] ="/sonar_ranger";

unsigned long pingTimer;


void setup() {
  // init the publisher node
  nh.initNode();

  // register a publisher for each topic for each sensors
  nh.advertise(pub_sonar);

  pingTimer = millis();
}

void loop() {
  if(millis()>=pingTimer){
    // Read From sensors
    sonarDist = sonar.ping_cm(); 

    String data = String(sonarDist);
    int length = data.length();
    char data_final[length+1];
    data.toCharArray(data_final, length+1);
    sonar_msg.data = data_final;
    pub_sonar.publish(&sonar_msg);

    pingTimer = millis() + PING_INTERVAL;
  }

  // keep the ROS node up and running
  nh.spinOnce();
}