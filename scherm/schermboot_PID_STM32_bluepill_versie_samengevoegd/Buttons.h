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



void button_state_change_reset() {
  buttonStateChange_enc_1 = false;
  buttonStateChange_enc_2 = false;
  buttonStateChange1 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange2 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange3 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange4 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange = false;   // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
}

// begin loop
//===================================================================== computeButtonPress =========================================================================

void computeButtonPress() {
  if (button_encoder_2 && buttonStateChange_enc_2) {
    switch (Globals::menu) {

      case Globals::Menu::OFF:
        Globals::menu = Globals::Menu::VOORVLEUGEL;
        break;

      case Globals::Menu::VOORVLEUGEL:
        Globals::menu = Globals::Menu::ACHTERVLEUGEL;
        break;

      case Globals::Menu::ACHTERVLEUGEL:
        Globals::menu = Globals::Menu::BALANS_VOORVLEUGEL;
        break;

      case Globals::Menu::BALANS_VOORVLEUGEL:
        Globals::menu = Globals::Menu::DEBUG;
        break;

      case Globals::Menu::DEBUG:
        Globals::menu = Globals::Menu::OFF;
        break;

      case Globals::Menu::STARTUP:
        break;
    }
  }
  if (cursorPlace == 6) {
    cursorPlace = 0;
  }
  if ((buttonAll == 1) && (menu == Menu::VOORVLEUGEL)) {  // works only when in V vl mode
    if ((button1 == HIGH) && (buttonStateChange1)) {
      cursorPlace--;
    } else if ((button2 == HIGH) && (buttonStateChange2)) {
      cursorPlace++;
      if (cursorPlace == 6) {
        cursorPlace = 0;
      }
    } else if ((button3 == HIGH) && (buttonStateChange3) && (cursorPlace == 0)) {  // if cursor place is at 0 change setDistance
      setDistance--;
    } else if ((button4 == HIGH) && (buttonStateChange4) && (cursorPlace == 0)) {
      setDistance++;
    } else if ((button3 == HIGH) && (buttonStateChange3) && (cursorPlace == 1)) {  // if cursor place is at 1 change roll setpoint
      setRoll--;
    } else if ((button4 == HIGH) && (buttonStateChange4) && (cursorPlace == 1)) {
      setRoll++;
    } else if ((button3 == HIGH) && (cursorPlace == 2)) {  // if cursor place is at 2 change the P from PID
      kp_Vvl--;
    } else if ((button4 == HIGH) && (cursorPlace == 2)) {
      kp_Vvl++;
    } else if ((button3 == HIGH) && (cursorPlace == 3)) {  // if cursor place is at 3 change the I from PID
      ki_Vvl--;
    } else if ((button4 == HIGH) && (cursorPlace == 3)) {
      ki_Vvl++;
    } else if ((button3 == HIGH) && (cursorPlace == 4)) {  // if cursor place is at 4 change the D from PID
      kd_Vvl--;
    } else if ((button4 == HIGH) && (cursorPlace == 4)) {
      kd_Vvl++;
    } else if ((button3 == HIGH) && (cursorPlace == 5)) {  // if cursor place is at 5 change the pitch van de boot
      setPitch--;
    } else if ((button4 == HIGH) && (cursorPlace == 5)) {
      setPitch++;
    }
    pidChangeDetection = setDistance + cursorPlace + kp_Vvl + ki_Vvl + kd_Vvl + setPitch + setRoll;

  } else if ((buttonAll == 1) && (menu == Menu::BALANS_VOORVLEUGEL)) {  // works only when in 4 balans mode
    if ((button1 == HIGH) && (buttonStateChange1)) {
      cursorPlace--;
    } else if ((button2 == HIGH) && (buttonStateChange2)) {
      cursorPlace++;
      if (cursorPlace == 6) {
        cursorPlace = 0;
      }
    } else if ((button3 == HIGH) && (buttonStateChange3) && (cursorPlace == 0)) {  // if cursor place is at 0 change setDistance
      setDistance--;
    } else if ((button4 == HIGH) && (buttonStateChange4) && (cursorPlace == 0)) {
      setDistance++;
    } else if ((button3 == HIGH) && (buttonStateChange3) && (cursorPlace == 1)) {  // if cursor place is at 1 change roll setpoint
      setRoll--;
    } else if ((button4 == HIGH) && (buttonStateChange4) && (cursorPlace == 1)) {
      setRoll++;
    } else if ((button3 == HIGH) && (cursorPlace == 2)) {  // if cursor place is at 2 change the P from PID
      kp_balans--;
    } else if ((button4 == HIGH) && (cursorPlace == 2)) {
      kp_balans++;
    } else if ((button3 == HIGH) && (cursorPlace == 3)) {  // if cursor place is at 3 change the I from PID
      ki_balans--;
    } else if ((button4 == HIGH) && (cursorPlace == 3)) {
      ki_balans++;
    } else if ((button3 == HIGH) && (cursorPlace == 4)) {  // if cursor place is at 4 change the D from PID
      kd_balans--;
    } else if ((button4 == HIGH) && (cursorPlace == 4)) {
      kd_balans++;
    } else if ((button3 == HIGH) && (cursorPlace == 5)) {  // if cursor place is at 5 change the ptich van de boot
      setPitch--;
    } else if ((button4 == HIGH) && (cursorPlace == 5)) {
      setPitch++;
    }
    pidChangeDetection = setDistance + cursorPlace + kp_balans + ki_balans + kd_balans + setPitch + setRoll;

  } else if ((buttonAll == 1) && (menu == Menu::ACHTERVLEUGEL)) {  // works only when in 5 achtervleugel mode
    if ((button1 == HIGH) && (buttonStateChange1)) {
      cursorPlace--;
    } else if ((button2 == HIGH) && (buttonStateChange2)) {
      cursorPlace++;
      if (cursorPlace == 6) {
        cursorPlace = 0;
      }
    } else if ((button3 == HIGH) && (buttonStateChange3) && (cursorPlace == 0)) {  // if cursor place is at 0 change setDistance
      setDistance--;
    } else if ((button4 == HIGH) && (buttonStateChange4) && (cursorPlace == 0)) {
      setDistance++;
    } else if ((button3 == HIGH) && (buttonStateChange3) && (cursorPlace == 1)) {  // if cursor place is at 1 change roll setpoint
      setRoll--;
    } else if ((button4 == HIGH) && (buttonStateChange4) && (cursorPlace == 1)) {
      setRoll++;
    } else if ((button3 == HIGH) && (cursorPlace == 2)) {  // if cursor place is at 2 change the P from PID
      kp_Avl--;
    } else if ((button4 == HIGH) && (cursorPlace == 2)) {
      kp_Avl++;
    } else if ((button3 == HIGH) && (cursorPlace == 3)) {  // if cursor place is at 3 change the I from PID
      ki_Avl--;
    } else if ((button4 == HIGH) && (cursorPlace == 3)) {
      ki_Avl++;
    } else if ((button3 == HIGH) && (cursorPlace == 4)) {  // if cursor place is at 4 change the D from PID
      kd_Avl--;
    } else if ((button4 == HIGH) && (cursorPlace == 4)) {
      kd_Avl++;
    } else if ((button3 == HIGH) && (cursorPlace == 5)) {  // if cursor place is at 5 change the pitch van de boot
      setPitch--;
    } else if ((button4 == HIGH) && (cursorPlace == 5)) {
      setPitch++;
    }
    pidChangeDetection = setDistance + cursorPlace + kp_Avl + ki_Avl + kd_Avl + setPitch + setRoll;
  }
  cursorPlace = constrain(cursorPlace, 0, 5);
  setDistance = constrain(setDistance, 0, 99);

  kp_Vvl = constrain(kp_Vvl, 0, 999);
  ki_Vvl = constrain(ki_Vvl, 0, 999);
  kd_Vvl = constrain(kd_Vvl, 0, 999);

  kp_Avl = constrain(kp_Avl, 0, 999);
  ki_Avl = constrain(ki_Avl, 0, 999);
  kd_Avl = constrain(kd_Avl, 0, 999);

  kp_balans = constrain(kp_balans, 0, 999);
  ki_balans = constrain(ki_balans, 0, 999);
  kd_balans = constrain(kd_balans, 0, 999);

  setPitch = constrain(setPitch, -99, 999);
}


}