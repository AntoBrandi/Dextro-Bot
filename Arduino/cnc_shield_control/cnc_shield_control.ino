#include <AccelStepper.h>

AccelStepper Xaxis(1, 2, 5); // pin 2 = step, pin 5 = direction

const byte enablePin = 8;

void setup()
{
   pinMode(enablePin, OUTPUT);
   digitalWrite(enablePin, LOW);

   Xaxis.setMaxSpeed(12800);
   Xaxis.setSpeed(1000); // had to slow for my motor
}

void loop()
{
   Xaxis.runSpeed();
}
