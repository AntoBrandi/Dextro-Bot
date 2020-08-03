#include <Aduino.h>
#include <NewPing.h>
#include <SimpleKalmanFilter.h>
#include <ros.h>
#include <sensor_msgs/Range.h>

// ARDUINO ULTRASONIC RANGE PUBLISHER
// This script will allow users to publish ultrasonic sensor data to ROS
// via Arduino formatted as sensor_msgs/Range message

#define SONAR_NUM 4 // number of ultrasonic sensors
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


// Variables
unsigned long nextPingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
unsigned long _timerStart = 0;
int LOOPING = 40; // Loop for every 40 milliseconds.
uint8_t oldSensorReading[SONAR_NUM];    //Store last valid value of the sensors.
 
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


// Functions
void initRangeMessage(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = FIELD_OF_VIEW;
  range_name.min_range = MIN_DISTANCE;
  range_name.max_range = MAX_DISTANCE/100;
}

// compare the current time in millis with the respect of the time
// when the loop started
bool isTimeForLoop(int _mSec) {
  return (millis() - _timerStart) > _mSec;
}

// reset the information stored in the time variable so that the loop can start
// again after a specified amount of time
void resetTimer() {
  _timerStart = millis();
}

// If ping received, set the sensor distance to array.
void echoCheck() {
  if (sonars[currentSensor].check_timer())
    cm[currentSensor] = sonars[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

// If sensor value is 0, then return the last stored value different than 0.
int returnLastValidRead(uint8_t sensorArray, uint8_t cm) {
  if (cm != 0) {
    return oldSensorReading[sensorArray] = cm;
  } else {
    return oldSensorReading[sensorArray];
  }
}
 
// Return the last valid value from the sensor.
void oneSensorCycle() {
  front_sonar   = returnLastValidRead(0, cm[0]);
  left_sonar   = returnLastValidRead(1, cm[1]);
  right_sonar = returnLastValidRead(2, cm[2]);
  back_sonar  = returnLastValidRead(3, cm[3]);
}

// looping the sensors
void sensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= nextPingTimer[i]) {
      nextPingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle();
      sonars[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonars[currentSensor].ping_timer(echoCheck);
    }
  }
}
 
// Apply Kalman Filter to sensor reading.
void applyKF() {
  front_sonar_filtered = KFilter.updateEstimate(front_sonar);
  left_sonar_filtered = KFilter.updateEstimate(left_sonar);
  right_sonar_filtered = KFilter.updateEstimate(right_sonar);
  back_sonar_filtered = KFilter.updateEstimate(back_sonar);
}


void setup() {
  // init the ping timer for each sonar that indicates the next ping time
  nextPingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    nextPingTimer[i] = nextPingTimer[i - 1] + PING_INTERVAL;
  
  // init the publisher node
  nh.initNode();

  // register a publisher for each topic for each sensors
  nh.advertise(pub_sonar_front);
  nh.advertise(pub_sonar_left);
  nh.advertise(pub_sonar_right);
  nh.advertise(pub_sonar_back);

  // init the sensor_msgs/Range message with common information between sensors
  initRangeMessage(sonar_front, TOPIC_SONAR_FRONT);
  initRangeMessage(sonar_left, TOPIC_SONAR_LEFT);
  initRangeMessage(sonar_right, TOPIC_SONAR_RIGHT);
  initRangeMessage(sonar_back, TOPIC_SONAR_BACK);
}

void loop() {
  if(isTimeForLoop(LOOPING)){
    sensorCycle();
    oneSensorCycle();
    applyKF();

    // compose the sensor_msgs/Range message
    sonar_front.range = front_sonar_filtered;
    sonar_left.range = left_sonar_filtered;
    sonar_right.range = right_sonar_filtered;
    sonar_back.range = back_sonar_filtered;

    sonar_front.header.stamp = nh.now();
    sonar_left.header.stamp = nh.now();
    sonar_right.header.stamp = nh.now();
    sonar_back.header.stamp = nh.now();

    // publish each sonar reading on each topic
    pub_sonar_front.publish(&sonar_front);
    pub_sonar_left.publish(&sonar_left);
    pub_sonar_right.publish(&sonar_right);
    pub_sonar_back.publish(&sonar_back);


    resetTimer();
  }


  // keep the ROS node up and running
  nh.spinOnce();
}