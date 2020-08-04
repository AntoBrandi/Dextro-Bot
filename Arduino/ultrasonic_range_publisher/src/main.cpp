#include <Arduino.h>
#include <NewPing.h>
#include <SimpleKalmanFilter.h>
#include <ros.h>
#include <sensor_msgs/Range.h>

// ARDUINO ULTRASONIC RANGE PUBLISHER
// This script will allow users to publish ultrasonic sensor data to ROS
// via Arduino formatted as sensor_msgs/Range message

#define SONAR_NUM 4 // number of ultrasonic sensors
#define PI 3.1416
#define MIN_DISTANCE 0
#define MAX_DISTANCE 200 // Max distance for obstacle detection
#define FIELD_OF_VIEW 0.26
#define PING_INTERVAL 33 // Milliseconds between two consequent readings

// Arduino Pins
// Front sonar
#define TRIGGER_FRONT_SONAR 3
#define ECHO_FRONT_SONAR 2
// Left sonar
#define TRIGGER_LEFT_SONAR 5
#define ECHO_LEFT_SONAR 4
// Right sonar
#define TRIGGER_RIGHT_SONAR 7
#define ECHO_RIGHT_SONAR 6
// Back sonar
#define TRIGGER_BACK_SONAR 9
#define ECHO_BACK_SONAR 8

// Kalman Filter parameters
#define KALMAN_E_MEA 2
#define KALMAN_E_EST 2
#define KALMAN_Q 0.01

// Topic names
#define TOPIC_SONAR_FRONT "/sonar_front"
#define TOPIC_SONAR_LEFT "/sonar_left"
#define TOPIC_SONAR_RIGHT "/sonar_right"
#define TOPIC_SONAR_BACK "/sonar_back"

// Register the ultrasonic sensors as instances of the class NewPing 
// that will hide the low level logic to extract sensor readings
NewPing sonars[SONAR_NUM] = {
  // Front
  NewPing(TRIGGER_FRONT_SONAR, ECHO_FRONT_SONAR, MAX_DISTANCE), // Trigger pin, echo pin, and max distance to ping.
  // Left
  NewPing(TRIGGER_LEFT_SONAR, ECHO_LEFT_SONAR, MAX_DISTANCE),
  // Right
  NewPing(TRIGGER_RIGHT_SONAR, ECHO_RIGHT_SONAR, MAX_DISTANCE),
  // Back
  NewPing(TRIGGER_BACK_SONAR, ECHO_BACK_SONAR, MAX_DISTANCE)
};
 

// Create a Kalman Filter that will be applied to the readings of all the sonar sensors
// e_mea: Measurement Uncertainty
// e_est: Estimation Uncertainty
// q: Process Noise
SimpleKalmanFilter KFilter(KALMAN_E_MEA, KALMAN_E_EST, KALMAN_Q);

// Init the ROS node
ros::NodeHandle nh;

// Message that will be created and published on ros
sensor_msgs::Range sonar_front;
sensor_msgs::Range sonar_left;
sensor_msgs::Range sonar_right;
sensor_msgs::Range sonar_back;
 
// Topics on which the sonar messages will be published
ros::Publisher pub_sonar_front(TOPIC_SONAR_FRONT, &sonar_front);
ros::Publisher pub_sonar_left(TOPIC_SONAR_LEFT, &sonar_left);
ros::Publisher pub_sonar_right(TOPIC_SONAR_RIGHT, &sonar_right);
ros::Publisher pub_sonar_back(TOPIC_SONAR_BACK, &sonar_back);
 
// Raw data coming from sensors
uint8_t front_sonar;
uint8_t left_sonar;             
uint8_t right_sonar;
uint8_t back_sonar;

// Filtered data coming from sensor and passed through a Kalmn Filter
uint8_t front_sonar_filtered;
uint8_t left_sonar_filtered;             
uint8_t right_sonar_filtered;
uint8_t back_sonar_filtered;

char frameid[] ="/sonar_ranger";


// Functions
void initRangeMessage(sensor_msgs::Range &range_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frameid;
  range_name.field_of_view = FIELD_OF_VIEW;
  range_name.min_range = MIN_DISTANCE;
  range_name.max_range = MAX_DISTANCE/100;
}
 
// Apply Kalman Filter to sensor reading.
void applyKF() {
  front_sonar_filtered = KFilter.updateEstimate(front_sonar);
  left_sonar_filtered = KFilter.updateEstimate(left_sonar);
  right_sonar_filtered = KFilter.updateEstimate(right_sonar);
  back_sonar_filtered = KFilter.updateEstimate(back_sonar);
}


void setup() {
  // init the publisher node
  nh.initNode();

  // register a publisher for each topic for each sensors
  nh.advertise(pub_sonar_front);
  nh.advertise(pub_sonar_left);
  nh.advertise(pub_sonar_right);
  nh.advertise(pub_sonar_back);

  // init the sensor_msgs/Range message with common information between sensors
  initRangeMessage(sonar_front);
  initRangeMessage(sonar_left);
  initRangeMessage(sonar_right);
  initRangeMessage(sonar_back);
}

void loop() {
  // Read From sensors
  // Front sonar
  front_sonar = sonars[0].ping_cm(); 
  front_sonar_filtered = KFilter.updateEstimate(front_sonar);
  // Left sonar
  left_sonar = sonars[1].ping_cm(); 
  left_sonar_filtered = KFilter.updateEstimate(front_sonar);
  // Right sonar
  right_sonar = sonars[2].ping_cm(); 
  right_sonar_filtered = KFilter.updateEstimate(front_sonar);
  // Back sonar
  back_sonar = sonars[3].ping_cm(); 
  back_sonar_filtered = KFilter.updateEstimate(front_sonar);

  // compose the sensor_msgs/Range message
  sonar_front.range = (float)front_sonar_filtered/100;
  sonar_left.range = (float)left_sonar_filtered/100;
  sonar_right.range = (float)right_sonar_filtered/100;
  sonar_back.range = (float)back_sonar_filtered/100;

  sonar_front.header.stamp = nh.now();
  sonar_left.header.stamp = nh.now();
  sonar_right.header.stamp = nh.now();
  sonar_back.header.stamp = nh.now();

  // publish each sonar reading on each topic
  pub_sonar_front.publish(&sonar_front);
  pub_sonar_left.publish(&sonar_left);
  pub_sonar_right.publish(&sonar_right);
  pub_sonar_back.publish(&sonar_back);


  // keep the ROS node up and running
  nh.spinOnce();
}