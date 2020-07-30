#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/String.h>

// Define the PIN number of the Arduino board connected with the driver motors
// MOTOR 1
#define dirPin_1 2
#define stepPin_1 3
// MOTOR 2
#define dirPin_2 4
#define stepPin_2 5
// MOTOR 3
#define dirPin_3 6
#define stepPin_3 7
// MOTOR 4
#define dirPin_4 8
#define stepPin_4 9
// AccelStepper parameter
#define motorInterfaceType 1

// Stepper Motor control parameters
// Max speed of the steppers in steps per second
#define MAX_SPEED 1500
// Max acceleration of the steppers in steps per second ^2
#define MAX_ACCELERATION 1000
// Speed levels for the robot mouvements
#define SUPERSONIC 1500
#define INSANE 1000
#define SUPERFAST 800
#define FAST 600
#define NORMAL 400
#define SLOW 300

// Track the current action on the stepper
int action = 0;

// Function that is called once a message is published on the topic on which this Arduino is subscribed
void trigger(const std_msgs::String& msg){
  String ms = msg.data;
  // for debug
  if((ms == "A") || (ms == "a")){
    digitalWrite(13,HIGH);
    action = 1;
  }
  else if ((ms == "Q") || (ms == "q")){
    digitalWrite(13,LOW);
    action = 0;
  }
}

// Create an instance of the class AccelStepper for each stepper motor connected to the Arduino board
// via DRV8825 stepper driver
AccelStepper motor_1 = AccelStepper(motorInterfaceType, stepPin_1, dirPin_1);
AccelStepper motor_2 = AccelStepper(motorInterfaceType, stepPin_2, dirPin_2);
AccelStepper motor_3 = AccelStepper(motorInterfaceType, stepPin_3, dirPin_3);
AccelStepper motor_4 = AccelStepper(motorInterfaceType, stepPin_4, dirPin_4);

// Set this Arduino controller to be a ROS Node that can publish and subscribe to ROS topics
ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> sub("/stepper_trigger", trigger );

void setup() {
  // open the serial terminal for debug purposes
  pinMode(13,OUTPUT);
  
  // Set the maximum speed 
  motor_1.setMaxSpeed(MAX_SPEED);
  motor_2.setMaxSpeed(MAX_SPEED);
  motor_3.setMaxSpeed(MAX_SPEED);
  motor_4.setMaxSpeed(MAX_SPEED);

  // Set the maximum acceleration
  motor_1.setAcceleration(MAX_ACCELERATION);
  motor_2.setAcceleration(MAX_ACCELERATION);
  motor_3.setAcceleration(MAX_ACCELERATION);
  motor_4.setAcceleration(MAX_ACCELERATION);

  // cerate and initialize a ROS node on this Arduino controller
  nh.initNode();
  nh.subscribe(sub);
}
void loop() {
  // According to the last received message control the stepper
  switch(action){
    case 0:
      stopRobot();
      break;
    case 1:
      goForward(NORMAL);
      break;
    default:
      stopRobot();
      break;
  }
  // Keep ROS Node Up & Running
  nh.spinOnce();
}


// Omnidirectional wheels mouvements
// SPEED CONTROL
void goForward(int velocity){
    // Set the speed in steps per second:
  motor_1.setSpeed(velocity);
  motor_2.setSpeed(velocity);
  motor_3.setSpeed(velocity);
  motor_4.setSpeed(velocity);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();  
}

void goBackward(int velocity){
  // Set the speed in steps per second:
  motor_1.setSpeed(-velocity);
  motor_2.setSpeed(-velocity);
  motor_3.setSpeed(-velocity);
  motor_4.setSpeed(-velocity);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void goRight(int velocity){
  // Set the speed in steps per second:
  motor_1.setSpeed(velocity);
  motor_2.setSpeed(-velocity);
  motor_3.setSpeed(-velocity);
  motor_4.setSpeed(velocity);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void goLeft(int velocity){
  // Set the speed in steps per second:
  motor_1.setSpeed(-velocity);
  motor_2.setSpeed(velocity);
  motor_3.setSpeed(velocity);
  motor_4.setSpeed(-velocity);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void goForwardRight(int velocity){
  // Set the speed in steps per second:
  motor_1.setSpeed(velocity);
  motor_2.setSpeed(0);
  motor_3.setSpeed(0);
  motor_4.setSpeed(velocity);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void goForwardLet(int velocity){
  // Set the speed in steps per second:
  motor_1.setSpeed(0);
  motor_2.setSpeed(velocity);
  motor_3.setSpeed(velocity);
  motor_4.setSpeed(0);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void goBackwardRight(int velocity){
  // Set the speed in steps per second:
  motor_1.setSpeed(0);
  motor_2.setSpeed(-velocity);
  motor_3.setSpeed(-velocity);
  motor_4.setSpeed(0);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

void goBackwardLeft(int velocity){
  // Set the speed in steps per second:
  motor_1.setSpeed(-velocity);
  motor_2.setSpeed(0);
  motor_3.setSpeed(0);
  motor_4.setSpeed(-velocity);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}

// POSITION CONTROL
void moveRobotTo(int steps_1, int steps_2, int steps_3, int steps_4){
  // set a target position in steps for each motor
  motor_1.moveTo(steps_1);
  motor_2.moveTo(steps_2);
  motor_3.moveTo(steps_3);
  motor_4.moveTo(steps_4);
  // Reach the position goal
  motor_1.runToPosition();
  motor_2.runToPosition();
  motor_3.runToPosition();
  motor_4.runToPosition();
}

// Stop all the motors
void stopRobot(){
  // Set the speed in steps per second:
  motor_1.setSpeed(0);
  motor_2.setSpeed(0);
  motor_3.setSpeed(0);
  motor_4.setSpeed(0);
  // Step the motor with a constant speed as set by setSpeed():
  motor_1.runSpeed();
  motor_2.runSpeed();
  motor_3.runSpeed();
  motor_4.runSpeed();
}
