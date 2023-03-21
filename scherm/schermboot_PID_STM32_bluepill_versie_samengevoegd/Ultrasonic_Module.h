#include "variant_PILL_F103Cx.h"
#pragma once

#include "Globals.h"

//using namespace Globals;
namespace Ultrasonic_Module {

float pitch = 0;  // tijdelijk voor debuggen moet uit de CAN.h komen

// Pin assignment for Ultrasonic_Module
const uint8_t trig_1_pin = PB3;   // Pin number for trigger signal
const uint8_t echo_1_pin = PA15;  // Pin number for echo signal (interrupt pin)
const uint8_t trig_2_pin = PA12;  // Pin number for trigger signal 2, NOT USED
const uint8_t echo_2_pin = PA11;  // Pin number for echo signal 2 (interrupt pin), NOT USED

volatile uint32_t travelTime = 0;     // the time it takes the sound to comback to the sensor in micoseconds
volatile bool newMesurement = false;  // is true when the interupt is triggerd to indicate a new mesurement of een nieuwe gyro meting.
volatile bool newHightMesurement = false;
//const uint16_t   soundSpeed              = 343;              // Speed of sound in m/s (choos one soundspeed)
const float soundSpeed = 58.309038;  // speed of sound in micosecond/cm (29,15*2=58.3 want heen en terug)  (choos one soundspeed)
float pitch_rad;                     // arduino werkt in radians.
int16_t distance = 0;                // distance from de ultrasoic sensor in cm

// Functions have to be decalred before they can be used in another function (in this case  "attachInterrupt()") this is called a "forward declaration"
void call_INT0();

void setup_Ultrasonic_Module() {
  pinMode(trig_1_pin, OUTPUT);                                            // Pin 3 as trig_1_pin (output)
  pinMode(echo_1_pin, INPUT);                                             // Pin 2 [INT0] as echo_1_pin (input)
  attachInterrupt(digitalPinToInterrupt(echo_1_pin), call_INT0, CHANGE);  // Pin 2 -> INT0 // Attach function call_INT0 to pin 2 when it CHANGEs state
}

//=================================================================== doMeasurement =================================================================

void doMeasurement() {
  // Initiate next trigger
  digitalWrite(trig_1_pin, LOW);   // LOW
  delayMicroseconds(2);            // for 2µs
  digitalWrite(trig_1_pin, HIGH);  // HIGH
  delayMicroseconds(10);           // for 10µs
  digitalWrite(trig_1_pin, LOW);   // Set LOW again
}

//==================================================== interrupt for ultrasonic sensor ============================================================

// Interrupt handling for INT0 (pin 2 on Arduino Uno)
// Keep this as FAST and LIGHT (cpu load) as possible !
void call_INT0() {
  byte pinRead;
  pinRead = digitalRead(echo_1_pin);  // same as digitalRead(2) but faster

  unsigned long currentTime = micros();  // Get current time (in µs)
  static volatile uint32_t startTime;
  static volatile uint32_t oldTravelTime = 0;
  if (pinRead) {
    // If pin state has changed to HIGH -> remember start time (in µs)
    startTime = currentTime;
  } else {
    // If pin state has changed to LOW -> calculate time passed (in µs)
    travelTime = currentTime - startTime;
    // Serial.println(travelTime / 58.3);
    newMesurement = true;
    newHightMesurement = true;

    //=========================================================================== out of range detection =========================================================================

    static uint8_t i = 0;
    if ((travelTime > 15000) || (travelTime < 1300)) {  // if object is out of range or mesurement error
      travelTime = oldTravelTime;                       // use preveus mesurement becaus of mesurement error
      if ((i < 4) && (travelTime > 1300)) {             // count out of range mesurement untill five
        i++;
      } else {
        travelTime = 0;  // if object is realy out of range return 0
      }
    } else {
      oldTravelTime = travelTime;  // save a new mesurement as old mesurment
      i = 0;                       // reset out of range counter
    }
  }
}

//============================================================= computeDistance ===================================================================
void computeDistance() {
  static int16_t distance_sensor;
  distance_sensor = (travelTime + (0.5 * soundSpeed)) / soundSpeed;  // afstand in cm. because int are rounded down we add 0,5 cm or 29 micoseconds

  pitch_rad = pitch * Globals::pi / 180.0;                           // degees to radians
  distance = distance_sensor - (270 * tan(pitch_rad)) - 40.0 + 0.5;  // afstand van de onderkant van de boot (-40) tot het water onder de vleugel door rekening te houden met de hoek van de boot(-270*tan(pitch_rad). because int are rounded down we add 0,5
}


}  // namespace Ultrasonic_Module