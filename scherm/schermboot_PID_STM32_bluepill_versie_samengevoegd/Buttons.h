#pragma once

namespace Buttons {

const uint8_t buttonPin4 = BUTTON_1;  // Pin number for button
const uint8_t buttonPin3 = BUTTON_2;  // Pin number for button
const uint8_t buttonPin2 = BUTTON_3;  // Pin number for button
const uint8_t buttonPin1 = BUTTON_4;  // Pin number for button
const uint8_t buttonPin_encoder_1 = ENC_1_BTN;
const uint8_t buttonPin_encoder_2 = ENC_2_BTN;


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

void setup_Buttons() {
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