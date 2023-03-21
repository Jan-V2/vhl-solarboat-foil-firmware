//#include <Arduino.h>
#include "pinmap_bluepill.h"
#include "LCD_Module.h"
#include "CAN_Module.h"
#include "Ultrasonic_Module.h"
#include "Buttons.h"
#include "Globals.h"
#include "PID_Berekeningen.h"

//using namespace Globals;



//uint8_t controlMode = 0;          // 0 = off, 1 = manuel, 2 = Vvl, 3 = HOME, 4 = balans en 5 = Avl
uint8_t cursorPlace = 0;  // is used to select the parameter that you want to change when in PID controlmode



//RunningMedian travelTimeMedian = RunningMedian(medianSize);

void setup() {

  Serial.begin(115200);
  

  Ultrasonic_Module::setup_Ultrasonic_Module();

  CAN_Module::setup_CAN_Module();

  LCD_Module::setup_LCD_Module();
  
  Buttons::setup_buttons_encoders();

  delay(25);
  while (Buttons::buttonAll == 0) {
    Buttons::buttonPressDetection();  // wait until button press
    delay(25);
  }
  while (Buttons::buttonAll == 1) {
    Buttons::buttonPressDetection();  // wait until button release
    delay(25);
  }
  LCD_Module::lcd.clear();

  Globals::startupMenu();

  while (!Serial) {
    ;  //Wait for Serial
  }
  Serial.println F(("--- Serial monitor started ---"));
}
void loop() {

  CAN_Module::CAN_Module_loop();


  //================================================================== main loop poll sensor ==========================================================================

  
Ultrasonic_Module::Ultrasonic_Module_loop();

  static uint32_t lastPollButtons = 0;
  if (millis() - lastPollButtons > Buttons::pollTimeButtons) {
    lastPollButtons = millis();
    Buttons::buttonPressDetection();  // check for buttonpress
  }

  //================================================================== main loop compute data ==========================================================================

  static uint8_t x;
  static uint32_t lastButtonCompute = 0;

  if (Buttons::buttonStateChange) {  // check if button was just pressed
    x = 255;                // delay before longpress starts
  }
  if ((millis() - lastButtonCompute > x) || (Buttons::buttonStateChange)) {  // normal compute delay or longpess delay
    lastButtonCompute = millis();
    Globals::computeButtonPress();
    if (!Buttons::buttonStateChange) {  // if no state change the button is still pressed
      x = Buttons::buttonCompompute;    // reset delay to (longpress) normal delay
    }
  }
  
 PID_Berekeningen::PID_Berekeningen_loop();

  //================================================================== main loop display data ==========================================================================

 LCD_Module::LCD_Module_loop();

  //===================================================================== main loop reset buttonStateChange ============================================================
  Buttons::button_state_change_reset();
  
}










//======================================================================= displayControlMode ==========================================================================





//=========================================================================== startupMenu =========================================================================



