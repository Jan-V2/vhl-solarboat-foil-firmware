//#include <Arduino.h>
#include "pinmap_bluepill.h"
#include "LCD.h"
#include "CAN_Module.h"
#include "Ultrasonic_Module.h"
#include "Buttons.h"
#include "Globals.h"

using namespace Globals;

enum class Menu : uint8_t {
  OFF,
  VOORVLEUGEL,
  ACHTERVLEUGEL,
  BALANS_VOORVLEUGEL,
  DEBUG,
  STARTUP
};

Menu menu;




const uint16_t PID_compute_time = 250;                               // How many milliseconds between PID compute.
const uint16_t maxPulseEncoder = 11487;                              // the maximum amount of pulses for the front foil motor encoder
const uint16_t maxAfstandEncoder = 200;                              // de afstand in mm die de voor linieare motor kan uit schuiven
const uint16_t pulsen_per_mm = maxPulseEncoder / maxAfstandEncoder;  // pulsen per mm van de linieare motor
const int16_t minDistance = 5;                                       // als de boot onder de minimale hoogte komt dan wordt de hoek van de vleugel aggresiever.
const int16_t maxDistance = 30;                                      // als de boot boven de maximale hoogte komt dan wordt de hoek van de vleugel minder aggresief.

//uint8_t controlMode = 0;          // 0 = off, 1 = manuel, 2 = Vvl, 3 = HOME, 4 = balans en 5 = Avl
uint8_t cursorPlace = 0;  // is used to select the parameter that you want to change when in PID controlmode

int16_t kp_Vvl = 0;               // P parameter from the PID voorvleugel
int16_t ki_Vvl = 0;               // I parameter from the PID voorvleugel
int16_t kd_Vvl = 0;               // D parameter from the PID voorvleugel
int16_t kp_Avl = 0;               // P parameter from the PID voorvleugel
int16_t ki_Avl = 0;               // I parameter from the PID voorvleugel
int16_t kd_Avl = 0;               // D parameter from the PID voorvleugel
int16_t kp_balans = 0;            // P parameter from the PID voorvleugel
int16_t ki_balans = 0;            // I parameter from the PID voorvleugel
int16_t kd_balans = 0;            // D parameter from the PID voorvleugel
uint16_t pidChangeDetection = 0;  // is used to see if there are changes in the PID setting

const uint8_t pollTimeSensor = 89;  // How many milliseconds between sensor polls (the PID runs at the same speed)

float P_Vvl;
float I_Vvl;
float D_Vvl;
float P_Avl;
float I_Avl;
float D_Avl;
float P_Balans;
float I_Balans;
float D_Balans;
float pidVvlTotal = 0;
float pidAvlTotal = 0;
float pidBalansTotal = 0;

uint8_t setDistance = 10;              // target distance in cm that the PID will try to reach, this value can be changed on de
int8_t setRoll = 0;                    // target roll in 10de graden( 1 = 0,1 graden en 10 = 1 graad) that the PID will try to reach, this value can be changed on de
int16_t setPitch = 0;                  // target pitch in 10de graden( 1 = 0,1 graden en 10 = 1 graad) that the PID will try to reach, this value can be changed on de
int16_t pulsen_offset = 0;             // berekende pulsen offset

bool pid_actief = false;               // PID staat uit wanneer false. kan aangepast worden in OFF controlmode 0





#include <LiquidCrystal.h>
LiquidCrystal lcd(RS, E, D4, D5, D6, D7);
//RunningMedian travelTimeMedian = RunningMedian(medianSize);

void setup() {

  Serial.begin(115200);
  setup_buttons_and_encoders();

  Ultrasonic_Module::setup_Ultrasonic_Module();

  CAN_Module::setup_CAN_Module();

  LCD_Module::setup_LCD_Module();
  
  Buttons::setup_Buttons();

  delay(25);
  while (Buttons::buttonAll == 0) {
    buttonPressDetection();  // wait until button press
    delay(25);
  }
  while (Buttons::buttonAll == 1) {
    buttonPressDetection();  // wait until button release
    delay(25);
  }
  lcd.clear();

  startupMenu();

  while (!Serial) {
    ;  //Wait for Serial
  }
  Serial.println F(("--- Serial monitor started ---"));
}
void loop() {

  CAN_Module::CAN_loop();

  //================================================================== main loop poll sensor ==========================================================================

  static uint32_t lastPollSensor = 0;
  if (millis() - lastPollSensor > pollTimeSensor) {
    lastPollSensor = millis();
    Ultrasonic_Module::doMeasurement();  // measure the distance from the ultrasonic sensor
  }
  static uint32_t lastPollButtons = 0;
  if (millis() - lastPollButtons > Buttons::pollTimeButtons) {
    lastPollButtons = millis();
    buttonPressDetection();  // check for buttonpress
  }

  //================================================================== main loop compute data ==========================================================================

  static uint8_t x;
  static uint32_t lastButtonCompute = 0;

  if (Buttons::buttonStateChange) {  // check if button was just pressed
    x = 255;                // delay before longpress starts
  }
  if ((millis() - lastButtonCompute > x) || (Buttons::buttonStateChange)) {  // normal compute delay or longpess delay
    lastButtonCompute = millis();
    computeButtonPress();
    if (!buttonStateChange) {  // if no state change the button is still pressed
      x = Buttons::buttonCompompute;    // reset delay to (longpress) normal delay
    }
  }
  static uint32_t last_PID_compute_time = 0;
  static uint16_t lastPidChangeDetection = 1;
  if ((((millis() - last_PID_compute_time > PID_compute_time) || Ultrasonic_Module::newMesurement || (pidChangeDetection != lastPidChangeDetection)) && pid_actief)) {
    last_PID_compute_time = millis();
    Ultrasonic_Module::newMesurement = false;
    Ultrasonic_Module::computeDistance();
    computePid_Vvl();
    computePid_Avl();
    computePid_balans();

    Serial.print(millis() - last_PID_compute_time);
    Serial.print(" - ");
    Serial.print(PID_compute_time);
    Serial.print(" - ");
    Serial.print(Ultrasonic_Module::newMesurement);
    Serial.print(" - ");
    Serial.print(pidChangeDetection);
    Serial.print(" - ");
    Serial.print(lastPidChangeDetection);
    Serial.print(" - ");
    Serial.println(pid_actief);
  }

  //================================================================== main loop display data ==========================================================================

  static uint32_t lastRefreshDistanceDisplay = 0;
  if (millis() - lastRefreshDistanceDisplay > refreshDistanceDisplay) {
    lastRefreshDistanceDisplay = millis();
    Ultrasonic_Module::computeDistance();
    displayData();
    blink_cursor();
  }

  if ((pidChangeDetection != lastPidChangeDetection) && pid_actief) {  // wanneer de PID ingesteld word
    lastPidChangeDetection = pidChangeDetection;
    if (menu != Menu::DEBUG) {
      pidDisplay();
      blink_cursor();
    }
  }

  static Menu last_menu = Menu::STARTUP;  // use STARTUP so that it runs at least ones to display the data

  if (menu != last_menu) {
    last_menu = menu;
    displayControlMode();
    blink_cursor();
  }
  if (menu == Menu::OFF) {
    OFF();
  }
  if (menu == Menu::DEBUG) {
    home();
  }

  //===================================================================== main loop reset buttonStateChange ============================================================
  button_state_change_reset();
  
}

//============================================================================ pidDisplay ===============================================================

void pidDisplay() {
  if (cursorPlace == 0) {  // if 0 change the setDitance parameter
    lcd.setCursor(6, 0);
    lcd.print F((">"));
    if (setDistance < 10) {
      lcd.print F((" "));
    }
    lcd.print(setDistance);
    lcd.setCursor(9, 0);
    lcd.print F(("cm"));
  } else {
    lcd.setCursor(6, 0);
    lcd.print F(("S"));
  }
  if (cursorPlace == 1) {  // if 1 change the setRoll parameter
    lcd.setCursor(12, 1);
    lcd.print F((">"));
    if (setRoll < 0) {
      if (setRoll > -10) {
        lcd.print F((" "));
      }
    } else {
      lcd.print F((" "));  // spatie want geen negatief getal
      if (setRoll < 10) {
        lcd.print F((" "));
      }
    }
    lcd.print(setRoll);
  } else {
    lcd.setCursor(12, 1);
    lcd.print F(("R"));
  }
  if (cursorPlace == 5) {  // if 5 change the pitch hoek
    lcd.setCursor(7, 3);
    lcd.print F((">"));
    lcd.print(char(224));
    lcd.print(setPitch);
    if (setPitch < 10 && setPitch > -10) {
      lcd.print F((" "));
    }
    if (setPitch < 100 && setPitch >= 10) {
      lcd.print F((" "));
    }
  } else {
    lcd.setCursor(7, 3);
    lcd.print F(("S"));
    lcd.print(char(224));
    lcd.print(setPitch);
    if (setPitch < 10) {
      lcd.print F((" "));
    }
  }
  //===========================================
  if (menu == Menu::VOORVLEUGEL) {  // voor vleugel
    if (cursorPlace == 2) {         // if 2 change the P from the PID parameter
      lcd.setCursor(0, 1);
      lcd.print F((">"));
      lcd.print(kp_Vvl);
      if (kp_Vvl < 10) {
        lcd.print F(("  "));
      } else if (kp_Vvl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(0, 1);
      lcd.print F(("P"));
      lcd.print(kp_Vvl);
      if (kp_Vvl < 10) {
        lcd.print F(("  "));
      } else if (kp_Vvl < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 3) {  // if 3 change the I from the PID parameter
      lcd.setCursor(4, 1);
      lcd.print F((">"));
      lcd.print(ki_Vvl);
      if (ki_Vvl < 10) {
        lcd.print F(("  "));
      } else if (ki_Vvl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(4, 1);
      lcd.print F(("I"));
      lcd.print(ki_Vvl);
      if (ki_Vvl < 10) {
        lcd.print F(("  "));
      } else if (ki_Vvl < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 4) {  // if 4 change the D from the PID parameter
      lcd.setCursor(8, 1);
      lcd.print F((">"));
      lcd.print(kd_Vvl);
      if (kd_Vvl < 10) {
        lcd.print F(("  "));
      } else if (kd_Vvl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(8, 1);
      lcd.print F(("D"));
      lcd.print(kd_Vvl);
      if (kd_Vvl < 10) {
        lcd.print F(("  "));
      } else if (kd_Vvl < 100) {
        lcd.print F((" "));
      }
    }
  }
  if (menu == Menu::BALANS_VOORVLEUGEL) {  // balans
    if (cursorPlace == 2) {                // if 2 change the P from the PID parameter
      lcd.setCursor(0, 1);
      lcd.print F((">"));
      lcd.print(kp_balans);
      if (kp_balans < 10) {
        lcd.print F(("  "));
      } else if (kp_balans < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(0, 1);
      lcd.print F(("P"));
      lcd.print(kp_balans);
      if (kp_balans < 10) {
        lcd.print F(("  "));
      } else if (kp_balans < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 3) {  // if 3 change the I from the PID parameter
      lcd.setCursor(4, 1);
      lcd.print F((">"));
      lcd.print(ki_balans);
      if (ki_balans < 10) {
        lcd.print F(("  "));
      } else if (ki_balans < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(4, 1);
      lcd.print F(("I"));
      lcd.print(ki_balans);
      if (ki_balans < 10) {
        lcd.print F(("  "));
      } else if (ki_balans < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 4) {  // if 4 change the D from the PID parameter
      lcd.setCursor(8, 1);
      lcd.print F((">"));
      lcd.print(kd_balans);
      if (kd_balans < 10) {
        lcd.print F(("  "));
      } else if (kd_balans < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(8, 1);
      lcd.print F(("D"));
      lcd.print(kd_balans);
      if (kd_balans < 10) {
        lcd.print F(("  "));
      } else if (kd_balans < 100) {
        lcd.print F((" "));
      }
    }
  }
  if (menu == Menu::ACHTERVLEUGEL) {  // achtervleugel
    if (cursorPlace == 2) {           // if 2 change the P from the PID parameter
      lcd.setCursor(0, 1);
      lcd.print F((">"));
      lcd.print(kp_Avl);
      if (kp_Avl < 10) {
        lcd.print F(("  "));
      } else if (kp_Avl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(0, 1);
      lcd.print F(("P"));
      lcd.print(kp_Avl);
      if (kp_Avl < 10) {
        lcd.print F(("  "));
      } else if (kp_Avl < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 3) {  // if 3 change the I from the PID parameter
      lcd.setCursor(4, 1);
      lcd.print F((">"));
      lcd.print(ki_Avl);
      if (ki_Avl < 10) {
        lcd.print F(("  "));
      } else if (ki_Avl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(4, 1);
      lcd.print F(("I"));
      lcd.print(ki_Avl);
      if (ki_Avl < 10) {
        lcd.print F(("  "));
      } else if (ki_Avl < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 4) {  // if 4 change the D from the PID parameter
      lcd.setCursor(8, 1);
      lcd.print F((">"));
      lcd.print(kd_Avl);
      if (kd_Avl < 10) {
        lcd.print F(("  "));
      } else if (kd_Avl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(8, 1);
      lcd.print F(("D"));
      lcd.print(kd_Avl);
      if (kd_Avl < 10) {
        lcd.print F(("  "));
      } else if (kd_Avl < 100) {
        lcd.print F((" "));
      }
    }
  }
}

//===================================================================== computeButtonPress =========================================================================

void computeButtonPress() {
  if (button_encoder_2 && buttonStateChange_enc_2) {
    switch (menu) {

      case Menu::OFF:
        menu = Menu::VOORVLEUGEL;
        break;

      case Menu::VOORVLEUGEL:
        menu = Menu::ACHTERVLEUGEL;
        break;

      case Menu::ACHTERVLEUGEL:
        menu = Menu::BALANS_VOORVLEUGEL;
        break;

      case Menu::BALANS_VOORVLEUGEL:
        menu = Menu::DEBUG;
        break;

      case Menu::DEBUG:
        menu = Menu::OFF;
        break;

      case Menu::STARTUP:
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



//========================================================== compute pid voorvleugel ===============================================================
void computePid_Vvl() {
  static float error = 0;
  static float diffError = 0;
  static float diffErrorFilter = 0;
  static float oldError = 0;
  static uint32_t lastPidTime = 0;
  static uint32_t pidTime = 0;
  static int16_t pidLoopTime_ms = 0;
  static float pidLoopTime_s = 0;
  static float hoek_voor_vleugel = 0;
  static uint16_t pulsen_liniear = 0;
  static float afstand_liniear = 0;
  static float max_I_Vvl_new = 7.0 * 0.090;  // de motor kan de vleugel maximaal met 4,89 (5 in formule) graden per seconden verstellen en iedere 90ms wordt de pid opnieuw berekend
  static float min_I_Vvl = 0;
  static float max_I_Vvl = 0;


  if (Ultrasonic_Module::newHightMesurement == true) {
    Ultrasonic_Module::newHightMesurement = false;
    pidTime = millis();
    pidLoopTime_ms = pidTime - lastPidTime;
    lastPidTime = pidTime;
    pidLoopTime_s = float(pidLoopTime_ms) / 1000.0;
    error = float(setDistance) - float(Ultrasonic_Module::distance);
    diffError = error - oldError;
    oldError = error;

    diffErrorFilter = diffErrorFilter * 0.7 + diffError * 0.3;  // filter om te voorkomen dat de D te aggrasief reageert op ruis.

    P_Vvl = float(kp_Vvl) * error / 100.0;  // delen door 100 om komma te besparen op het display.
    if ((abs(CAN_Module::PWM_links) + abs(CAN_Module::PWM_rechts)) != 800) {
      static float I_Vvl_new = 0;
      I_Vvl_new = float(ki_Vvl) * error * pidLoopTime_s / 100.0;

      I_Vvl = I_Vvl + constrain(I_Vvl_new, -max_I_Vvl_new, max_I_Vvl_new);
    }
    D_Vvl = (float(kd_Vvl) * diffErrorFilter / pidLoopTime_s) / 100.0;

    P_Vvl = constrain(P_Vvl, -9.9, 12.0);
    I_Vvl = constrain(I_Vvl, -9.9, 12.0);
    D_Vvl = constrain(D_Vvl, -9.9, 9.9);

    min_I_Vvl = -9.9 - P_Vvl;
    max_I_Vvl = 12 - P_Vvl;
    I_Vvl = constrain(I_Vvl, min_I_Vvl, max_I_Vvl);

    if (ki_Vvl == 0) {
      I_Vvl = 0.0;
    }
    pidVvlTotal = P_Vvl + I_Vvl + D_Vvl;  // PID wordt berekend in graden
  }

  pidVvlTotal = constrain(pidVvlTotal, -9.9, 12.0);

  //if (Ultrasonic_Module::distance < minDistance) {
  //  pidVvlTotal = 4;
  //}
  //if (Ultrasonic_Module::distance > maxDistance) {
  //  pidVvlTotal = -3;
  //}

  //Serial.print("pidVvlTotal: ");
  //Serial.println(pidVvlTotal);
  hoek_voor_vleugel = pidVvlTotal - CAN_Module::pitch;
  //Serial.print("hoek_voor_vleugel: ");
  //Serial.println(hoek_voor_vleugel);
  afstand_liniear = (sqrt(43.2 * 43.2 + 17.2 * 17.2 - 2 * 43.2 * 17.2 * cos((hoek_voor_vleugel + 90.0 - 3) * pi / 180.0))) - 30.55;  // lengte linieare motor in cm is wortel(b^2+c^2 - 2*b*c*cos(hoek vleugel)) wanneer vleugel 0 graden is staat deze haaks op de boot dus 90graden. -3 omdat de vleugel hoger gemonteerd zit dan de linieare motor.
  //Serial.print("afstand_liniear: ");
  //Serial.println(afstand_liniear);
  pulsen_liniear = afstand_liniear * pulsen_per_mm * 10;  // pulsen naar voorvleugel = afstand in cm maal pulsen per cm
  //Serial.print("pulsen_liniear: ");
  //Serial.println(pulsen_liniear);
  CAN_Module::CAN_pulsen_voor = pulsen_liniear;
}

//============================================================================== compute pid achtervleugel========================================================================

void computePid_Avl() {
  static float error = 0;
  static float diffError = 0;
  static float diffErrorFilter = 0;
  static float oldError = 0;
  static uint32_t lastPidTime = 0;
  static uint32_t pidTime = 0;
  static int16_t pidLoopTime_ms = 0;
  static float pidLoopTime_s = 0;
  static float hoek_achter_vleugel = 0;
  static uint16_t pulsen_liniear = 0;
  static float hoek_home = -2.8;

  pidTime = millis();
  pidLoopTime_ms = pidTime - lastPidTime;
  lastPidTime = pidTime;
  pidLoopTime_s = float(pidLoopTime_ms) / 1000.0;
  error = CAN_Module::pitch - float(setPitch) / 10.0;  // f is het zelfde als .0
  diffError = error - oldError;
  oldError = error;
  diffErrorFilter = diffErrorFilter * 0.7 + diffError * 0.3;  // filter om te voorkomen dat de D te aggrasief reageert op ruis.

  P_Avl = float(kp_Avl) * error / 100.0;  // delen door 100 om komma te besparen op het display.
  if (abs(CAN_Module::PWM_achter) != 400) {
    I_Avl = I_Avl + (float(ki_Avl) * error * pidLoopTime_s / 100.0);
  }
  D_Avl = (float(kd_Avl) * float(diffErrorFilter) / pidLoopTime_s) / 100.0;

  P_Avl = constrain(P_Avl, hoek_home, 12.0);
  I_Avl = constrain(I_Avl, hoek_home, 12.0);
  D_Avl = constrain(D_Avl, hoek_home, 9.9);

  if (ki_Avl == 0) {
    I_Avl = 0.0;
  }
  pidAvlTotal = P_Avl + I_Avl + D_Avl;  // PID wordt berekend in graden

  pidAvlTotal = constrain(pidAvlTotal, hoek_home, 12.0);

  hoek_achter_vleugel = CAN_Module::pitch - pidAvlTotal;
  pulsen_liniear = (hoek_achter_vleugel - hoek_home) * 105.595;
  CAN_Module::CAN_pulsen_achter = pulsen_liniear;
  Serial.print(CAN_Module::CAN_pulsen_achter);
}

//======================================================================== PID offset ===========================================================================

void computePid_balans() {
  static float error = 0;
  static float diffError = 0;
  static float diffErrorFilter = 0;
  static float oldError = 0;
  static uint32_t lastPidTime = 0;
  static uint32_t pidTime = 0;
  static int16_t pidLoopTime_ms = 0;
  static float pidLoopTime_s = 0;
  static float offset_voor_vleugel = 0;
  static uint16_t pulsen_liniear;

  pidTime = millis();
  pidLoopTime_ms = pidTime - lastPidTime;
  lastPidTime = pidTime;
  pidLoopTime_s = float(pidLoopTime_ms) / 1000.0;
  error = float(setRoll) / 10.0 - CAN_Module::roll;
  diffError = error - oldError;
  oldError = error;
  diffErrorFilter = diffErrorFilter * 0.7 + diffError * 0.3;  // filter om te voorkomen dat de D te aggrasief reageert op ruis.

  P_Balans = float(kp_balans) * error / 100.0;  // delen door 100 om komma te besparen op het display.
  if ((abs(CAN_Module::PWM_links) + abs(CAN_Module::PWM_rechts)) != 800) {
    I_Balans = I_Balans + (float(ki_balans) * error * pidLoopTime_s / 100.0);
  }
  D_Balans = (float(kd_balans) * float(diffErrorFilter) / pidLoopTime_s) / 100.0;

  P_Balans = constrain(P_Balans, -5, 5);
  I_Balans = constrain(I_Balans, -5, 5);
  D_Balans = constrain(D_Balans, -5, 5);

  if (ki_balans == 0) {
    I_Balans = 0.0;
  }
  pidBalansTotal = P_Balans + I_Balans + D_Balans;  // PID wordt berekend in graden

  pidBalansTotal = constrain(pidBalansTotal, -5, 5);  // max 10mm offset

  offset_voor_vleugel = pidBalansTotal;                  // offset is in mm
  pulsen_liniear = offset_voor_vleugel * pulsen_per_mm;  // mm naar pulsen
  CAN_Module::CAN_pulsen_offset = pulsen_liniear;
}

//======================================================================= displayData ==========================================================================

void displayData() {
  lcd.setCursor(0, 0);  // set curser at distance place
  int x constrain(Ultrasonic_Module::distance, -99, 999);
  if (x == -39) {             // check for error
    lcd.print F(("ERROR "));  // print error
  } else {                    // if no error print the distance
    if ((x >= 0) && (x < 100)) {
      lcd.print F((" "));
      if (x < 10) {
        lcd.print F((" "));
      }
    } else if ((x > -10) && (x < 0)) {
      lcd.print F((" "));
    }
    lcd.print(x);
    lcd.print F(("cm"));  // print unit cm for distance
  }
  if (menu != Menu::DEBUG) {
    lcd.setCursor(16, 3);
    lcd.print(char(224));
    if (CAN_Module::pitch > -1 && CAN_Module::pitch < 10) {
      lcd.print F((" "));
      if (CAN_Module::pitch >= 0 && CAN_Module::pitch < 1) {
        lcd.print F((" "));
      }
    }
  }
  float pitch_display = CAN_Module::pitch * 10;
  pitch_display = constrain(pitch_display, -99, 999);
  lcd.print(pitch_display, 0);

  switch (menu) {
      /*
    online status: vvl & avl controller & homed?, gyroscoop, telemetrie en telemetrie online, gashendel en temperatuursensor
    */
    case Menu::DEBUG:
      lcd.setCursor(13, 0);  // set curser at debug vvl place
      lcd.print F(("vvl: "));
      if (CAN_Module::status_Vvl) {
        lcd.write(1);
      } else {
        lcd.write(3);
      }

      lcd.setCursor(13, 1);  // set curser at debug avl place
      lcd.print F(("avl: "));
      if (CAN_Module::status_Avl) {
        lcd.write(1);
      } else {
        lcd.write(3);
      }

      lcd.setCursor(6, 0);  // set curser at debug gyroscoop place
      lcd.print F(("gyro:"));
      if (CAN_Module::status_gryo) {
        lcd.write(1);
      } else {
        lcd.write(3);
      }

      lcd.setCursor(0, 1);  // set curser at debug gashendel place
      lcd.print F(("Gas:"));
      if (CAN_Module::status_gashendel) {
        lcd.write(1);
      } else {
        lcd.write(3);
      }

      lcd.setCursor(9, 2);  // set curser at homing vvl place
      lcd.print F(("HomeVvl:"));

      if (!CAN_Module::home_front_foil && !CAN_Module::has_homed_voor_vleugel) {  // if not homed
        lcd.write(3);
      } else if (CAN_Module::home_front_foil) {  // if homing
        lcd.write(2);
      } else if (!CAN_Module::home_front_foil && CAN_Module::has_homed_voor_vleugel) {  // if homed
        lcd.write(1);
      }

      lcd.setCursor(9, 3);  // set curser at homing vvl place
      lcd.print F(("HomeAvl:"));
      if (!CAN_Module::home_rear_foil && !CAN_Module::has_homed_achter_vleugel) {  // if not homed
        lcd.write(3);
      } else if (CAN_Module::home_rear_foil) {  // if homing
        lcd.write(2);
      } else if (!CAN_Module::home_rear_foil && CAN_Module::has_homed_achter_vleugel) {  // if homed
        lcd.write(1);
      }

      lcd.setCursor(6, 1);  // set curser at debug tempratuur sensor
      lcd.print F(("temp:"));
      if (CAN_Module::status_temp) {
        lcd.write(1);
      } else {
        lcd.write(3);
      }

      lcd.setCursor(0, 3);  // status telemetrie server
      lcd.print F(("tlmWWW:"));
      if (CAN_Module::status_telemetrie_verbinding_server) {
        lcd.write(1);
      } else {
        lcd.write(3);
      }

      lcd.setCursor(0, 2);  // status telemetrie CAN
      lcd.print F(("tlmCAN:"));
      if (CAN_Module::status_telemetrie_verbinding_CAN) {
        lcd.write(1);
      } else {
        lcd.write(3);
      }

      break;

    case Menu::VOORVLEUGEL:  // controlMode voorvleugel
      lcd.setCursor(0, 2);   // print P_Vvl
      lcd.print("P");
      if (P_Vvl >= 0) {
        lcd.print F((" "));
        if (P_Vvl < 1) {
          lcd.print(" ");
        }
      } else if (P_Vvl > -1.0) {
        lcd.print(" ");
      }
      lcd.print(P_Vvl * 10.0, 0);

      lcd.setCursor(4, 2);  // print I_Vvl
      lcd.print(" I");
      if (I_Vvl >= 0) {
        lcd.print F((" "));
        if (I_Vvl < 1) {
          lcd.print(" ");
        }
      } else if (I_Vvl > -1.0) {
        lcd.print(" ");
      }
      lcd.print(I_Vvl * 10.0, 0);

      lcd.setCursor(9, 2);  // print D_Vvl
      lcd.print(" D");
      if (D_Vvl >= 0.0) {
        lcd.print F((" "));
        if (D_Vvl < 1.0) {
          lcd.print(" ");
        }
      } else if (D_Vvl > -1.0) {
        lcd.print(" ");
      }
      lcd.print(D_Vvl * 10.0, 0);

      lcd.setCursor(0, 3);  // print PID_Vvl
      lcd.print("PID");
      if ((pidVvlTotal < 10.0) && (pidVvlTotal >= 0.0)) {
        lcd.print(" ");
        if (pidVvlTotal < 1) {
          lcd.print(" ");
        }
      }
      if (pidVvlTotal < 0.0 && pidVvlTotal > -1.0) {
        lcd.print(" ");
      }
      lcd.print(pidVvlTotal * 10.0, 0);
      lcd.print(' ');
      break;

    case Menu::ACHTERVLEUGEL:  // controlMode achtervleugel
      lcd.setCursor(0, 2);     // print P_Avl
      lcd.print("P");
      if (P_Avl >= 0) {
        lcd.print F((" "));
        if (P_Avl < 1) {
          lcd.print(" ");
        }
      } else if (P_Avl > -1.0) {
        lcd.print(" ");
      }

      lcd.setCursor(4, 2);  // print I_Avl
      lcd.print(" I");
      if (I_Avl >= 0) {
        lcd.print F((" "));
        if (I_Avl < 1) {
          lcd.print(" ");
        }
      } else if (I_Avl > -1.0) {
        lcd.print(" ");
      }
      lcd.print(I_Avl * 10.0, 0);

      lcd.setCursor(9, 2);  // print D_Avl
      lcd.print(" D");
      if (D_Avl >= 0.0) {
        lcd.print F((" "));
        if (D_Avl < 1.0) {
          lcd.print(" ");
        }
      } else if (D_Avl > -1.0) {
        lcd.print(" ");
      }
      lcd.print(D_Avl * 10.0, 0);

      lcd.setCursor(0, 3);  // print PID_Avl
      lcd.print("PID");
      if ((pidAvlTotal < 10.0) && (pidAvlTotal >= 0.0)) {
        lcd.print(" ");
        if (pidAvlTotal < 1) {
          lcd.print(" ");
        }
      }
      if (pidAvlTotal < 0.0 && pidAvlTotal > -1.0) {
        lcd.print(" ");
      }
      lcd.print(pidAvlTotal * 10.0, 0);
      lcd.print(' ');
      break;

    case Menu::BALANS_VOORVLEUGEL:  // controlMode balansvleugel
      lcd.setCursor(0, 2);          // print P_Vvl
      lcd.print("P");
      if (P_Balans >= 0) {
        lcd.print F((" "));
        if (P_Balans < 1) {
          lcd.print(" ");
        }
      } else if (P_Balans > -1.0) {
        lcd.print(" ");
      }
      lcd.print(P_Balans * 10.0, 0);

      lcd.setCursor(4, 2);  // print I_Vvl
      lcd.print(" I");
      if (I_Balans >= 0) {
        lcd.print F((" "));
        if (I_Balans < 1) {
          lcd.print(" ");
        }
      } else if (I_Balans > -1.0) {
        lcd.print(" ");
      }
      lcd.print(I_Balans * 10.0, 0);

      lcd.setCursor(9, 2);  // print D_Balans
      lcd.print(" D");
      if (D_Balans
          >= 0.0) {
        lcd.print F((" "));
        if (D_Balans < 1.0) {
          lcd.print(" ");
        }
      } else if (D_Balans > -1.0) {
        lcd.print(" ");
      }
      lcd.print(D_Balans * 10.0, 0);

      lcd.setCursor(0, 3);  // print PID_Vvl
      lcd.print("PID");
      if ((pidBalansTotal < 10.0) && (pidBalansTotal >= 0.0)) {
        lcd.print(" ");
        if (pidBalansTotal < 1) {
          lcd.print(" ");
        }
      }
      if (pidBalansTotal < 0.0 && pidBalansTotal > -1.0) {
        lcd.print(" ");
      }
      lcd.print(pidBalansTotal * 10.0, 0);
      lcd.print(' ');
      break;

    case Menu::OFF:
      break;

    case Menu::STARTUP:
      break;
  }
}

//======================================================================= displayControlMode ==========================================================================

void displayControlMode() {
  pidChangeDetection++;
  lcd.setCursor(16, 0);
  if (menu == Menu::OFF) {  // check controlmode. for off, manuel or automatic PID control
    lcd.clear();
    lcd.setCursor(16, 0);
    lcd.print F((" OFF"));
  } else if (menu == Menu::VOORVLEUGEL) {
    lcd.print F(("V vl"));
  } else if (menu == Menu::DEBUG) {
    lcd.clear();
  } else if (menu == Menu::BALANS_VOORVLEUGEL) {
    lcd.print F(("Ball"));
  } else if (menu == Menu::ACHTERVLEUGEL) {
    lcd.print F(("A vl"));
  }
}

void OFF() {
  if (menu == Menu::OFF) {
    if (button_encoder_1 && buttonStateChange_enc_1) {
      pid_actief = !pid_actief;
      CAN_Module::PID_debug_telemetry = pid_actief;
    }
    lcd.setCursor(16, 0);
    if (pid_actief) {
      lcd.print("!");
    } else {
      lcd.print(" ");
    }
  }
}

void home() {
  const static uint16_t min_home_time = 3000;
  static uint32_t last_home_time = millis();

  if (menu == Menu::DEBUG) {
    if (button_encoder_1 && buttonStateChange_enc_1) {
      // lcd.setCursor(0, 1);
      // lcd.print("Home voor");
      CAN_Module::home_front_foil = true;
      pid_actief = false;
      CAN_Module::CAN_pulsen_voor = 0;
      CAN_Module::CAN_pulsen_offset = 0;
      last_home_time = millis();
    } else if (millis() - last_home_time > min_home_time) {
      CAN_Module::home_front_foil = false;
    }
    if (button_encoder_1 && buttonStateChange_enc_1) {
      Serial.println("homeing");
      //lcd.setCursor(0, 2);
      //lcd.print("Home achter");
      CAN_Module::home_rear_foil = true;
      pid_actief = false;
      CAN_Module::CAN_pulsen_achter = 0;
      last_home_time = millis();
    } else if (millis() - last_home_time > min_home_time) {
      CAN_Module::home_rear_foil = false;
    }
  }
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



void blink_cursor() {
  static bool blinkCursor = false;
  static bool prevBlinkCursor = blinkCursor;

  if (((menu == Menu::VOORVLEUGEL) || (menu == Menu::BALANS_VOORVLEUGEL) || (menu == Menu::ACHTERVLEUGEL)) && (pid_actief == true)) {
    if (cursorPlace == 0) {  // set hoogte
      lcd.setCursor(6, 0);
    } else if (cursorPlace == 1) {  // set roll
      lcd.setCursor(12, 1);
    } else if (cursorPlace == 2) {  // P
      lcd.setCursor(0, 1);
    } else if (cursorPlace == 3) {  // I
      lcd.setCursor(4, 1);
    } else if (cursorPlace == 4) {  // D
      lcd.setCursor(8, 1);
    } else if (cursorPlace == 5) {  // set pitch
      lcd.setCursor(7, 3);
    }
    blinkCursor = true;
  } else {
    blinkCursor = false;
  }
  if (blinkCursor != prevBlinkCursor) {
    prevBlinkCursor = blinkCursor;
    if (blinkCursor) {
      lcd.blink();
    } else {
      lcd.noBlink();
    }
  }
}
//=========================================================================== startupMenu =========================================================================

void startupMenu() {
  lcd.print F(("1:OFF"));  // print Menu
  lcd.setCursor(0, 1);
  lcd.print F(("2:Manuel"));
  lcd.setCursor(9, 0);
  lcd.print F(("3:V vl"));
  lcd.setCursor(9, 1);
  lcd.print F(("4:HOME"));
  lcd.setCursor(0, 2);
  lcd.print F(("5:Balans"));
  lcd.setCursor(9, 2);
  lcd.print F(("6:A vl"));

  while (buttonAll == 0) {
    buttonPressDetection();  // wait until button press
    delay(25);
  }
  if (button1) {
    menu = Menu::OFF;  // contolMode OFF
    lcd.clear();
  } else {
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print F(("Hold button"));
    delay(1000);
  }
  buttonPressDetection();
  delay(25);
  buttonPressDetection();

  if (button3) {
    menu = Menu::VOORVLEUGEL;  // contolMode V vl
  } else if (button4) {
    menu = Menu::DEBUG;  // contolMode Home
  } else if (button_encoder_1) {
    menu = Menu::BALANS_VOORVLEUGEL;  // controlMode Balans
  } else if (button_encoder_2) {
    menu = Menu::ACHTERVLEUGEL;  // controMode A vl
  }

  lcd.setCursor(1, 0);
  lcd.print F(("Release button"));  //  as feedback for menue selection

  while (buttonAll > 0) {
    buttonPressDetection();  // wait until button release
    delay(25);
  }
  lcd.clear();
}
