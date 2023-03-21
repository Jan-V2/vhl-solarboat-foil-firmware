//#include <Arduino.h>
#include "pinmap_bluepill.h"
#include "LCD_Module.h"
#include "CAN_Module.h"
#include "Ultrasonic_Module.h"
#include "Buttons.h"
#include "Globals.h"

using namespace Globals;



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
    Globals::computeButtonPress();
    if (!Buttons::buttonStateChange) {  // if no state change the button is still pressed
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

 LCD_Module::LCD_Module_loop();

  //===================================================================== main loop reset buttonStateChange ============================================================
  Buttons::button_state_change_reset();
  
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


//=========================================================================== startupMenu =========================================================================



