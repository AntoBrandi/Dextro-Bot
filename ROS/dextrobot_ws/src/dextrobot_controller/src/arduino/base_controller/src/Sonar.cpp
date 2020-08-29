/*
  Sonar.cpp - DextroBot
  Class that cointains the implementation of a sonar sensor 
  connected to the Arduino that will sense the enviroment and 
  compose ROS messages for the communication
  Copyright (c) 2020 Antonio Brandi.  All right reserved.
*/

#include <Sonar.h>


Sonar::Sonar(uint8_t triggerPin, uint8_t echoPin) : triggerPin(triggerPin), echoPin(echoPin)
{
    /* initialize random seed: */
    srand (time(NULL));
    /* generate secret number between 1 and 10: */
    int offset = rand() % 50 + 1;
    lastScan = millis()+offset;
}

Sonar::~Sonar()
{
}

void Sonar::sense(){
    if (millis() >= lastScan){
        // Read From sensors
        distance = sonar.ping_cm();

        // update the last scan time 
        lastScan = millis() + PING_INTERVAL;
    }   
}

String Sonar::composeStringMessage(){
    String data = String((float)distance/100);
    return data;
}