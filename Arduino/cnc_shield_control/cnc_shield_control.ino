#include <AccelStepper.h>

AccelStepper Xaxis(1, 12, 13); // pin 2 = step, pin 5 = direction
AccelStepper Yaxis(1, 2, 5); // pin 2 = step, pin 5 = direction
AccelStepper Zaxis(1, 3, 6); // pin 2 = step, pin 5 = direction
AccelStepper Aaxis(1, 4, 7); // pin 2 = step, pin 5 = direction

const byte enablePin = 8;

void setup()
{
   pinMode(enablePin, OUTPUT);
   digitalWrite(enablePin, LOW);

   Xaxis.setMaxSpeed(12800);
   Xaxis.setSpeed(600); // had to slow for my motor
   Yaxis.setMaxSpeed(12800);
   Yaxis.setSpeed(600); // had to slow for my motor
   Zaxis.setMaxSpeed(12800);
   Zaxis.setSpeed(600); // had to slow for my motor
   Aaxis.setMaxSpeed(12800);
   Aaxis.setSpeed(600); // had to slow for my motor
}

void loop()
{
   Xaxis.runSpeed();
   Yaxis.runSpeed();
   Zaxis.runSpeed();
   Aaxis.runSpeed();
}
