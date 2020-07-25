#include <AccelStepper.h>

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
#define FAST 600
#define NORMAL 400
#define SLOW 200

// Create an instance of the class AccelStepper for each stepper motor connected to the Arduino board
// via DRV8825 stepper driver
AccelStepper motor_1 = AccelStepper(motorInterfaceType, stepPin_1, dirPin_1);
AccelStepper motor_2 = AccelStepper(motorInterfaceType, stepPin_2, dirPin_2);
AccelStepper motor_3 = AccelStepper(motorInterfaceType, stepPin_3, dirPin_3);
AccelStepper motor_4 = AccelStepper(motorInterfaceType, stepPin_4, dirPin_4);

void setup() {
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
}
void loop() {
  // *** MOVE BY STEP POSITION ***
  // Set target position:
  // stepper.moveTo(32768);
  // Run to position with set speed and acceleration:
  // stepper.runToPosition();

  // *** MOVE BY CRUISE SPEED ***
  // Set the speed in steps per second:
  // stepper.setSpeed(400);
  // Step the motor with a constant speed as set by setSpeed():
  // stepper.runSpeed();
  
  
}


// Omnidirectional wheels mouvements
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

void goLeft(int velocty){
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

void goForwardRight(int veloity){
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
