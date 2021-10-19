//#include <Arduino.h>
#include "pinmap_bluepill.h"
#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(PB0);

const uint8_t triggerPin = trig_1;    // Pin number for trigger signal
const uint8_t echoPin = echo_1;       // Pin number for echo signal (interrupt pin)
const uint8_t buttonPin4 = BUTTON_1;  // Pin number for button
const uint8_t buttonPin3 = BUTTON_2;  // Pin number for button
const uint8_t buttonPin2 = BUTTON_3;  // Pin number for button
const uint8_t buttonPin1 = BUTTON_4;  // Pin number for button
const uint8_t buttonPin_encoder_1 = ENC_1_BTN;
const uint8_t buttonPin_encoder_2 = ENC_2_BTN;
const uint8_t pollTimeSensor = 89;  // How many milliseconds between sensor polls (the PID runs at the same speed)
//const uint16_t   soundSpeed              = 343;              // Speed of sound in m/s (choos one soundspeed)
const float soundSpeed = 58.309038;                                  // speed of sound in micosecond/cm (29,15*2=58.3 want heen en terug)  (choos one soundspeed)
const uint16_t refreshDistanceDisplay = 399;                         // How many milliseconds between display updates
const uint8_t pollTimeButtons = 24;                                  // How many milliseconds between button polls
const uint8_t buttonCompompute = 49;                                 // How many milliseconds between button compute. less mili is faster long press
const uint8_t sendCanTime = 10;                                      // How many milliseconds between sending CAN frames
const uint16_t PID_compute_time = 250;                               // How many milliseconds between PID compute.
const uint16_t maxPulseEncoder = 17008;                              // the maximum amount of pulses for the front foil motor encoder
const uint16_t maxAfstandEncoder = 203;                              // de afstand in mm die de voor linieare motor kan uit schuiven
const uint16_t pulsen_per_mm = maxPulseEncoder  / maxAfstandEncoder;  // pulsen per mm van de linieare motor

volatile uint32_t travelTime = 0;     // the time it takes the sound to comback to the sensor in micoseconds
int16_t distance = 0;                 // distance from de ultrasoic sensor in cm
volatile bool newMesurement = false;  // is true when the interupt is triggerd to indicate a new mesurement of een nieuwe gyro meting.
volatile bool newHightMesurement = false;
uint8_t controlMode = 0;              // 0 = off, 1 = manuel, 2 = Vvl, 3 = HOME, 4 = balans en 5 = Avl
uint8_t cursorPlace = 0;              // is used to select the parameter that you want to change when in PID controlmode
uint16_t pidChangeDetection = 0;      // is used to see if there are changes in the PID setting
int16_t kp_Vvl = 0;                   // P parameter from the PID voorvleugel
int16_t ki_Vvl = 0;                   // I parameter from the PID voorvleugel
int16_t kd_Vvl = 0;                   // D parameter from the PID voorvleugel
int16_t kp_Avl = 0;                   // P parameter from the PID voorvleugel
int16_t ki_Avl = 0;                   // I parameter from the PID voorvleugel
int16_t kd_Avl = 0;                   // D parameter from the PID voorvleugel
int16_t kp_balans = 0;                // P parameter from the PID voorvleugel
int16_t ki_balans = 0;                // I parameter from the PID voorvleugel
int16_t kd_balans = 0;                // D parameter from the PID voorvleugel
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
uint8_t setDistance = 10;           // target distance in cm that the PID will try to reach, this value can be changed on de
int8_t setRoll = 0;                 // target roll in 10de graden( 1 = 0,1 graden en 10 = 1 graad) that the PID will try to reach, this value can be changed on de
int16_t setPitch = 0;               // target pitch in 10de graden( 1 = 0,1 graden en 10 = 1 graad) that the PID will try to reach, this value can be changed on de
int16_t pulsen_offset = 0;          // berekende pulsen offset
int16_t leftAcutatorStroke = 150;   // amount of mm the actuator is extened. value is 150 so you dont have to press the button 1000x. the vlalue will only be send when the controllmode is != OFF
int16_t rightAcutatorStroke = 150;  // amount of mm the actuator is extened. value is 150 so you dont have to press the button 1000x. the vlalue will only be send when the controllmode is != OFF
int16_t leftAcutatorStroke2 = 150;
int16_t rightAcutatorStroke2 = 150;
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
bool pid_actief = false;               // PID staat uit wanneer false. kan aangepast worden in OFF controlmode 0

// data van CAN
float pitch;
float roll;
uint8_t overcurrent_achter;
int16_t PWM_links;
int16_t PWM_rechts;
int16_t PWM_achter;

int16_t CAN_pulsen_voor = 0;
int16_t CAN_pulsen_offset = 0;
int16_t CAN_pulsen_achter = 0;

bool home_front_foil;
bool home_rear_foil;

#include <LiquidCrystal.h>
LiquidCrystal lcd(RS, E, D4, D5, D6, D7);
//RunningMedian travelTimeMedian = RunningMedian(medianSize);

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  setup_buttons_and_encoders();

  lcd.begin(16, 4);  // Switch on the LCD screen
  lcd.setCursor(2, 0);
  lcd.print F(("VHL-Nordwin"));  // Print these words to my LCD screen
  lcd.setCursor(1, 2);
  lcd.print F(("Zonnebootteam"));

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  pinMode(triggerPin, OUTPUT);  // Pin 3 as triggerpin (output)
  pinMode(echoPin, INPUT);      // Pin 2 [INT0] as echopin (input)
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);
  pinMode(buttonPin_encoder_1, INPUT_PULLUP);

  // Attach function call_INT0 to pin 2 when it CHANGEs state
  attachInterrupt(digitalPinToInterrupt(echoPin), call_INT0, CHANGE);  // Pin 2 -> INT0

  delay(25);
  while (buttonAll == 0) {
    buttonPressDetection();  // wait until button press
    delay(25);
  }
  while (buttonAll == 1) {
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
  digitalWrite(LED_BUILTIN, HIGH);
  /*
    // ================================ code van youtbe ===========================
    // Has rotary encoder moved?
    if (rotaryEncoder) {
      // Get the movement (if valid)
      int8_t rotationValue = checkRotaryEncoder();

      // If valid movement, do something
      if (rotationValue != 0) {
        enc_1_pulses += rotationValue;
      }
    }
    // =============================== einde code youtube ============================
  */
  //================================================================== main loop poll sensor ==========================================================================

  static uint32_t lastPollSensor = 0;
  if (millis() - lastPollSensor > pollTimeSensor) {
    lastPollSensor = millis();
    doMeasurement();  // measure the distance from the ultrasonic sensor
  }
  static uint32_t lastPollButtons = 0;
  if (millis() - lastPollButtons > pollTimeButtons) {
    lastPollButtons = millis();
    buttonPressDetection();  // check for buttonpress
  }

  //================================================================== main loop compute data ==========================================================================

  static uint8_t x;
  static uint32_t lastButtonCompute = 0;

  if (buttonStateChange) {  // check if button was just pressed
    x = 255;                // delay before longpress starts
  }
  if ((millis() - lastButtonCompute > x) || (buttonStateChange)) {  // normal compute delay or longpess delay
    lastButtonCompute = millis();
    computeButtonPress();
    if (!buttonStateChange) {  // if no state change the button is still pressed
      x = buttonCompompute;    // reset delay to (longpress) normal delay
    }
  }
  static uint32_t last_PID_compute_time = 0;
  static uint16_t lastPidChangeDetection = 1;
  if ((((millis() - last_PID_compute_time > PID_compute_time) || newMesurement || (pidChangeDetection != lastPidChangeDetection)) && pid_actief)) {
    last_PID_compute_time = millis();
    newMesurement = false;
    computeDistance();
    computePid_Vvl();
    computePid_Avl();
    computePid_balans();
  }

  //================================================================== main loop display data ==========================================================================

  static uint32_t lastRefreshDistanceDisplay = 0;
  if (millis() - lastRefreshDistanceDisplay > refreshDistanceDisplay) {
    lastRefreshDistanceDisplay = millis();
    computeDistance();
    displayData();
    blink_cursor();
  }

  if ((pidChangeDetection != lastPidChangeDetection) && pid_actief) {  // wanneer de PID ingesteld word
    lastPidChangeDetection = pidChangeDetection;
    pidDisplay();
    blink_cursor();
  }

  static uint8_t lastControlMode = 255;  // use 255 so that it runs at least ones to display the data

  if (controlMode != lastControlMode) {
    lastControlMode = controlMode;
    displayControlMode();
    blink_cursor();
  }
  if (controlMode == 0) {
    OFF();
  }
  if (controlMode == 3) {
    home();
  }

  read_CAN_data();

  static uint32_t lastSendCan = 0;
  if (millis() - lastSendCan > sendCanTime) {
    lastSendCan = millis();
    send_CAN_data();
  }

  //===================================================================== main loop reset buttonStateChange ============================================================

  buttonStateChange_enc_1 = false;
  buttonStateChange_enc_2 = false;
  buttonStateChange1 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange2 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange3 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange4 = false;  // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
  buttonStateChange = false;   // reset buttonStateChange at the end of the loop if removed the numbers increase with two instead of one
}

//======================================================================== read_CAN_data ======================================================================

void read_CAN_data() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == 0x64) {
      pitch = float_from_can(0);  // byte 0-3 is float pitch
      roll = float_from_can(4);   // byte 4-7 is float roll
      newMesurement = true;       // er is een nieuwe meting voor de PID compute
      // Serial.println(pitch);
    } else if (canMsg.can_id == 0x32) {
      PWM_links = int16_from_can(canMsg.data[0], canMsg.data[1]);   //byte 0-1 is int16_t PWM links
      PWM_rechts = int16_from_can(canMsg.data[2], canMsg.data[3]);  //byte 0-1 is int16_t PWM rechts

    } else if (canMsg.can_id == 0x33) {
      PWM_achter = int16_from_can(canMsg.data[0], canMsg.data[1]);  //byte 0-1 is int16_t PWM achter
      overcurrent_achter = canMsg.data[2];                          //byte 2 is uint8_t overcurrent achter uint8_t
    }
  }
}

//========================================================================= send_CAN_data ==================================================================

void send_CAN_data() {
  if (! home_front_foil && ! home_rear_foil && pid_actief) {
    int_to_frame_thrice(CAN_pulsen_voor, CAN_pulsen_offset, CAN_pulsen_achter, 200);
    //  Serial.println(CAN_pulsen_voor);
  }
  if (home_front_foil) {
    bool_to_frame(home_front_foil, 301);  // TODO can ID toevoegen
  }
  if (home_rear_foil) {
    bool_to_frame(home_rear_foil, 300);  // TODO can ID toevoegen
  }
}

//========================================================================= doMeasurement =====================================================================

void doMeasurement() {
  // Initiate next trigger
  digitalWrite(triggerPin, LOW);   // LOW
  delayMicroseconds(2);            // for 2µs
  digitalWrite(triggerPin, HIGH);  // HIGH
  delayMicroseconds(10);           // for 10µs
  digitalWrite(triggerPin, LOW);   // Set LOW again
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
  if (controlMode == 2) {    // voor vleugel
    if (cursorPlace == 2) {  // if 2 change the P from the PID parameter
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
  if (controlMode == 4) {    // balans
    if (cursorPlace == 2) {  // if 2 change the P from the PID parameter
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
  if (controlMode == 5) {    // achtervleugel
    if (cursorPlace == 2) {  // if 2 change the P from the PID parameter
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
  static int8_t differnce = 0;
  static int prev_enc_1_pulses;
  static int prev_enc_2_pulses;
  differnce = leftAcutatorStroke - rightAcutatorStroke;
  //Serial.print(enc_1_pulses);
  /*
    if (enc_1_pulses < prev_enc_1_pulses) {
      controlMode--;
    } else if (enc_1_pulses > prev_enc_1_pulses) {
      controlMode++;
    }
  */
  prev_enc_1_pulses = enc_1_pulses;

  /*if (enc_2_pulses < prev_enc_2_pulses) {
    controlMode--; //cursorPlace--;
    } else if (enc_2_pulses > prev_enc_2_pulses) {
    controlMode++; //cursorPlace++;
    }*/
  if (button_encoder_2 && buttonStateChange_enc_2) {
    controlMode++;
  }
  prev_enc_2_pulses = enc_2_pulses;
  if (cursorPlace == 6) {
    cursorPlace = 0;
  }
  if (controlMode == 6) {
    controlMode = 0;
  }
  if ((buttonAll == 1) && (controlMode == 1)) {  // works only in manuel
    if (button1 == HIGH) {
      leftAcutatorStroke++;
      rightAcutatorStroke++;
    } else if (button2 == HIGH) {
      leftAcutatorStroke--;
      rightAcutatorStroke--;
    } else if ((button3 == HIGH) && (buttonStateChange3)) {
      if (differnce < 9) {
        leftAcutatorStroke++;
        rightAcutatorStroke--;
      }
    } else if ((button4 == HIGH) && (buttonStateChange4)) {
      if (differnce > -9) {
        leftAcutatorStroke--;
        rightAcutatorStroke++;
      }
    }
  } else if ((buttonAll == 1) && (controlMode == 2)) {  // works only when in V vl mode
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

  } else if ((buttonAll == 1) && (controlMode == 4)) {  // works only when in 4 balans mode
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

  } else if ((buttonAll == 1) && (controlMode == 5)) {  // works only when in 5 achtervleugel mode
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
  controlMode = constrain(controlMode, 0, 5);
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

//======================================================================= computeDistance ==========================================================================

void computeDistance() {
  static int16_t distance_sensor;
  distance_sensor = (travelTime + (0.5 * soundSpeed)) / soundSpeed;  // afstand in cm. because int are rounded down we add 0,5 cm or 29 micoseconds
  static float pitch_rad;                                            // arduino werkt in radians.
  pitch_rad = pitch * M_PI / 180.0;                                  // degees to radians
  distance = distance_sensor - (270 * tan(pitch_rad)) - 40.0 + 0.5;  // afstand van de onderkant van de boot (-40) tot het water onder de vleugel door rekening te houden met de hoek van de boot(-270*tan(pitch_rad). because int are rounded down we add 0,5
}

//======================================================================= compute pid voorvleugel ==========================================================================

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
  static uint16_t pulsen_liniear;
  static float afstand_liniear;

  if (newHightMesurement == true) {
    newHightMesurement = false;
    pidTime = millis();
    pidLoopTime_ms = pidTime - lastPidTime;
    lastPidTime = pidTime;
    pidLoopTime_s = float(pidLoopTime_ms) / 1000.0;
    error = float(setDistance) - float(distance);
    diffError = error - oldError;
    oldError = error;
    
    diffErrorFilter = diffErrorFilter * 0.7 + diffError * 0.3;  // filter om te voorkomen dat de D te aggrasief reageert op ruis.

    P_Vvl = float(kp_Vvl) * error / 100.0;  // delen door 100 om komma te besparen op het display.
    if ((abs(PWM_links) + abs(PWM_rechts)) != 800) {
      static float I_Vvl_new = 0;
      (float(ki_Vvl) * error * pidLoopTime_s / 100.0);
      I_Vvl = I_Vvl + constrain(I_Vvl_new, -max_I_Vvl_new, max_I_Vvl_new);
    }
    D_Vvl = (float(kd_Vvl) * diffErrorFilter / pidLoopTime_s) / 100.0;

    P_Vvl = constrain(P_Vvl, -9.9, 12.0);
    I_Vvl = constrain(I_Vvl, -9.9, 12.0);
    D_Vvl = constrain(D_Vvl, -9.9, 9.9);

    if (ki_Vvl == 0) {
      I_Vvl = 0.0;
    }
    pidVvlTotal = P_Vvl + I_Vvl + D_Vvl;  // PID wordt berekend in graden
  }

  pidVvlTotal = constrain(pidVvlTotal, -9.9, 12.0);
  //Serial.print("pidVvlTotal: ");
  //Serial.println(pidVvlTotal);
  hoek_voor_vleugel = pidVvlTotal - pitch;
  //Serial.print("hoek_voor_vleugel: ");
  //Serial.println(hoek_voor_vleugel);
  afstand_liniear = (sqrt(43.2 * 43.2 + 17.2 * 17.2 - 2 * 43.2 * 17.2 * cos((hoek_voor_vleugel + 90.0 - 3) * M_PI / 180.0))) - 30.55; // lengte linieare motor in cm is wortel(b^2+c^2 - 2*b*c*cos(hoek vleugel)) wanneer vleugel 0 graden is staat deze haaks op de boot dus 90graden. -3 omdat de vleugel hoger gemonteerd zit dan de linieare motor.
  //Serial.print("afstand_liniear: ");
  //Serial.println(afstand_liniear);
  pulsen_liniear = afstand_liniear * pulsen_per_mm * 10;                                                                                // pulsen naar voorvleugel = afstand in cm maal pulsen per cm
  //Serial.print("pulsen_liniear: ");
  //Serial.println(pulsen_liniear);
  CAN_pulsen_voor = pulsen_liniear;
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
  error = pitch - float(setPitch) / 10.0;  // f is het zelfde als .0
  diffError = error - oldError;
  oldError = error;
  diffErrorFilter = diffErrorFilter * 0.7 + diffError * 0.3;  // filter om te voorkomen dat de D te aggrasief reageert op ruis.

  P_Avl = float(kp_Avl) * error / 100.0;  // delen door 100 om komma te besparen op het display.
  if (abs(PWM_achter) != 400) {
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

  hoek_achter_vleugel = pitch - pidAvlTotal ;
  pulsen_liniear = (hoek_achter_vleugel - hoek_home) * 105.595;
  CAN_pulsen_achter = pulsen_liniear;
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
  error = float(setRoll) / 10.0 - roll;
  diffError = error - oldError;
  oldError = error;
  diffErrorFilter = diffErrorFilter * 0.7 + diffError * 0.3;  // filter om te voorkomen dat de D te aggrasief reageert op ruis.

  P_Balans = float(kp_balans) * error / 100.0;  // delen door 100 om komma te besparen op het display.
  if ((abs(PWM_links) + abs(PWM_rechts)) != 800) {
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
  CAN_pulsen_offset = pulsen_liniear;
}

//======================================================================= displayData ==========================================================================

void displayData() {
  lcd.setCursor(0, 0);  // set curser at distantce place
  int x constrain(distance, -99, 999);
  if (x == -39) {              // check for error
    lcd.print F(("ERROR "));  // print error
  } else {                   // if no error print the distance
    if ( (x >= 0) && (x < 100)) {
      lcd.print F((" "));
      if (x < 10) {
        lcd.print F((" "));
      }
    } else if ( (x > -10) && (x < 0)) {
      lcd.print F((" "));
    }
    lcd.print(x);
    lcd.print F(("cm"));  // print unit cm for distance
  }
  lcd.setCursor(12, 3);
  lcd.print (char(224));
  if (pitch > -1 && pitch < 10) {
    lcd.print F((" "));
    if (pitch >= 0 && pitch < 1) {
      lcd.print F((" "));
    }
  }
  float pitch_display = pitch * 10 ;
  pitch_display = constrain(pitch_display, -99, 999);
  lcd.print (pitch_display, 0);

  if (controlMode == 2) {  // controlMode voorvleugel
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
  }
  if (controlMode == 5) {  // controlMode achtervleugel
    lcd.setCursor(0, 2);   // print P_Avl
    lcd.print("P");
    if (P_Avl >= 0) {
      lcd.print F((" "));
      if (P_Avl < 1) {
        lcd.print(" ");
      }
    } else if (P_Avl > -1.0) {
      lcd.print(" ");
    }
    lcd.print(P_Avl * 10.0, 0);

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
  }
  if (controlMode == 4) {  // controlMode balansvleugel
    lcd.setCursor(0, 2);   // print P_Vvl
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
  }
}

//======================================================================= displayControlMode ==========================================================================

void displayControlMode() {
  pidChangeDetection++;
  lcd.setCursor(12, 0);
  if (controlMode == 0) {  // check controlmode. for off, manuel or automatic PID control
    lcd.clear();
    lcd.setCursor(12, 0);
    lcd.print F((" OFF"));
  } else if (controlMode == 1) {
    lcd.clear();
    lcd.setCursor(12, 0);
    lcd.print F(("MAN "));
  } else if (controlMode == 2) {
    lcd.print F(("V vl"));
  } else if (controlMode == 3) {
    lcd.clear();
    lcd.setCursor(12, 0);
    lcd.print F(("Home"));
  } else if (controlMode == 4) {
    lcd.print F(("Ball"));
  } else if (controlMode == 5) {
    lcd.print F(("A vl"));
  }
}

void OFF() {
  if (controlMode == 0) {
    if (button_encoder_1 && buttonStateChange_enc_1) {
      pid_actief = !pid_actief;
    }
    lcd.setCursor(12, 0);
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

  if (controlMode == 3) {
    if (button_encoder_1 && buttonStateChange_enc_1) {
      lcd.setCursor(0, 1);
      lcd.print("Home voor");
      home_front_foil = true;
      pid_actief = false;
      CAN_pulsen_voor = 0;
      CAN_pulsen_offset = 0;
      last_home_time = millis();
    } else if (millis() - last_home_time > min_home_time) {
      home_front_foil = false;
    }
    if (button_encoder_1 && buttonStateChange_enc_1) {
      lcd.setCursor(0, 2);
      lcd.print("Home achter");
      home_rear_foil = true;
      pid_actief = false;
      CAN_pulsen_achter = 0;
      last_home_time = millis();
    } else if (millis() - last_home_time > min_home_time) {
      home_rear_foil = false;
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

//=============================================================================== interrupt for ultrasonic sensor ==================================================================

// Interrupt handling for INT0 (pin 2 on Arduino Uno)
// Keep this as FAST and LIGHT (cpu load) as possible !
void call_INT0() {
  byte pinRead;
  pinRead = digitalRead(echoPin);  // same as digitalRead(2) but faster

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

void blink_cursor() {
  static bool blinkCursor = false;
  static bool prevBlinkCursor = blinkCursor;

  if (((controlMode == 2) || (controlMode == 4) || (controlMode == 5)) && (pid_actief == true)) {
    if (cursorPlace == 0) {
      lcd.setCursor(6, 0);
    } else if (cursorPlace == 1) {
      lcd.setCursor(12, 1);
    } else if (cursorPlace == 2) {
      lcd.setCursor(0, 1);
    } else if (cursorPlace == 3) {
      lcd.setCursor(4, 1);
    } else if (cursorPlace == 4) {
      lcd.setCursor(8, 1);
    } else if (cursorPlace == 5) {
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
    controlMode = 0;  // contolMode OFF
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

  if (button2) {
    controlMode = 1;  // contolMode Manuel
  } else if (button3) {
    controlMode = 2;  // contolMode V vl
  } else if (button4) {
    controlMode = 3;  // contolMode Home
  } else if (button_encoder_1) {
    controlMode = 4;  // controlMode Balans
  } else if (button_encoder_2) {
    controlMode = 5;  // controMode A vl
  }

  lcd.setCursor(1, 0);
  lcd.print F(("Release button"));  //  as feedback for menue selection

  while (buttonAll > 0) {
    buttonPressDetection();  // wait until button release
    delay(25);
  }
  lcd.clear();
}

float float_from_can(uint8_t start_idx) {
  byte byteVal[sizeof(float)];
  for (uint i = 0; i < sizeof(float); i++) {
    byteVal[i] = canMsg.data[start_idx + i];
  }
  float f;
  memcpy(&f, byteVal, sizeof(float));
  return f;
}

int16_t int16_from_can(uint8_t b1, uint8_t b2) {
  // maakt van twee bytes een int16_t
  int16_t ret;
  ret = b1 | (int16_t)b2 << 8;
  return ret;
}

can_frame int_to_frame_thrice(int16_t i16_1, int16_t i16_2, int16_t i16_3, uint16_t can_id) {
  byte bytes[sizeof(int16_t) * 3];
  memcpy(bytes, &i16_1, sizeof(int16_t));
  memcpy(bytes + sizeof(int16_t), &i16_2, sizeof(int16_t));
  memcpy(bytes + sizeof(int16_t) * 2, &i16_3, sizeof(int16_t));
  can_frame ret;
  for (uint8_t i = 0; i < sizeof(int16_t) * 3; i++) {
    ret.data[i] = bytes[i];
  }
  ret.can_id = can_id;
  ret.can_dlc = sizeof(int16_t) * 3;
  mcp2515.sendMessage(&ret);
  return ret;
}

can_frame bool_to_frame(bool b, uint16_t can_id) {
  byte bytes[sizeof(bool)];
  memcpy(bytes, &b, sizeof(bool));
  can_frame ret;
  for (uint8_t i = 0; i < sizeof(bool); i++) {
    ret.data[i] = bytes[i];
  }
  ret.can_id = can_id;
  ret.can_dlc = sizeof(bool);
  mcp2515.sendMessage(&ret);
  return ret;
}
