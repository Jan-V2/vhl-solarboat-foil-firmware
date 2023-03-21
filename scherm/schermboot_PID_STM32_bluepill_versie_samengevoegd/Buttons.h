#pragma once

namespace Buttons {

//input/buttons // KLOPT

#define ENC_1A PB9
#define ENC_1B PB8
#define ENC_2A PB15
#define ENC_2B PB14


int enc_1_pulses = 0;
int enc_2_pulses = 0;

const uint8_t buttonPin4 = PB4;  // Pin number for button
const uint8_t buttonPin3 = PB5;  // Pin number for button
const uint8_t buttonPin2 = PB6;  // Pin number for button
const uint8_t buttonPin1 = PB7;  // Pin number for button
const uint8_t buttonPin_encoder_1 = PA8;
const uint8_t buttonPin_encoder_2 = PB13;


const uint8_t pollTimeButtons = 24;   // How many milliseconds between button polls
const uint8_t buttonCompompute = 49;  // How many milliseconds between button compute. less mili is faster long press

uint8_t button1 = LOW;                 // LOW in rest state and HIGH when pressed
uint8_t button2 = LOW;                 // LOW in rest state and HIGH when pressed
uint8_t button3 = LOW;                 // LOW in rest state and HIGH when pressed
uint8_t button4 = LOW;                 // LOW in rest state and HIGH when pressed
uint8_t buttonAll = 0;                 // to count the total buttons that are high
uint8_t button_encoder_1 = LOW;        // LOW in rest state and HIGH when pressed
uint8_t button_encoder_2 = LOW;        // LOW in rest state and HIGH when pressed
bool buttonStateChange_enc_1 = false;  // is true if a button is recently changed its state
bool buttonStateChange_enc_2 = false;  // is true if a button is recently changed its state
bool buttonStateChange1 = false;       // is true if a button is recently changed its state
bool buttonStateChange2 = false;       // is true if a button is recently changed its state
bool buttonStateChange3 = false;       // is true if a button is recently changed its state
bool buttonStateChange4 = false;       // is true if a button is recently changed its state
bool buttonStateChange = false;        // is true if one of of the buttons has a state change. can be used as a flag to update the screen once before the refreshDisplay counter

void setup_Buttons();
void button_state_change_reset();
void buttonPressDetection();

void setup_buttons_encoders() {

  // The module already has pullup resistors on board
  // pinMode(ENC_1A, INPUT_PULLUP); // youtube
  // pinMode(ENC_1B, INPUT_PULLUP); // youtube

  
  // We need to monitor both pins, rising and falling for all states
  // attachInterrupt(digitalPinToInterrupt(ENC_1A), rotary, CHANGE); // youtube
  // attachInterrupt(digitalPinToInterrupt(ENC_1B), rotary, CHANGE); // youtube
  attachInterrupt(digitalPinToInterrupt(ENC_2A), encoder2_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_1A), encoder1_ISR, FALLING);
  
  pinMode(ENC_1_BTN, INPUT_PULLUP);
  pinMode(ENC_2_BTN, INPUT_PULLUP);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);
  pinMode(buttonPin_encoder_1, INPUT_PULLUP);
}// einde setup
// begin loop


void button_state_change_reset() {
  buttonStateChange_enc_1 = false;
  buttonStateChange_enc_2 = false;
  buttonStateChange1 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange2 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange3 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange4 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange = false;   // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
}


//======================================================================== buttonPressDetection =========================================================================

void buttonPressDetection() {
  button1 = !digitalRead(buttonPin1);
  button2 = !digitalRead(buttonPin2);
  button3 = !digitalRead(buttonPin3);
  button4 = !digitalRead(buttonPin4);
  button_encoder_1 = !digitalRead(buttonPin_encoder_1);
  button_encoder_2 = !digitalRead(buttonPin_encoder_2);

  buttonAll = button1 + button2 + button3 + button4 + button_encoder_1 + button_encoder_2;

  //=========================================================================== buttonStateChange detection =======================================================================

  static uint8_t lastButton1 = LOW;
  static uint8_t lastButton2 = LOW;
  static uint8_t lastButton3 = LOW;
  static uint8_t lastButton4 = LOW;
  static uint8_t lastButton_encoder_1 = LOW;
  static uint8_t lastButton_encoder_2 = LOW;

  if (lastButton1 != button1) {  // button1
    lastButton1 = button1;
    buttonStateChange1 = true;
  }
  if (lastButton2 != button2) {  // button2
    lastButton2 = button2;
    buttonStateChange2 = true;
  }
  if (lastButton3 != button3) {  // button3
    lastButton3 = button3;
    buttonStateChange3 = true;
  }
  if (lastButton4 != button4) {  // button4
    lastButton4 = button4;
    buttonStateChange4 = true;
  }
  if (lastButton_encoder_1 != button_encoder_1) {  // encoder button 1
    lastButton_encoder_1 = button_encoder_1;
    buttonStateChange_enc_1 = true;
  }
  if (lastButton_encoder_2 != button_encoder_2) {  // encoder button 2
    lastButton_encoder_2 = button_encoder_2;
    buttonStateChange_enc_2 = true;
  }
  if ((buttonStateChange1 + buttonStateChange2 + buttonStateChange3 + buttonStateChange4 + buttonStateChange_enc_1 + buttonStateChange_enc_2) > 0) {
    buttonStateChange = true;
  }
}

}