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
const uint16_t maxPulseEncoder = 13000;                              // the maximum amount of pulses for the front foil motor encoder
const uint16_t maxAfstandEncoder = 200;                              // de afstand in mm die de voor linieare motor kan uit schuiven
const uint16_t pulsen_per_mm = maxPulseEncoder / maxAfstandEncoder;  // pulsen per mm van de linieare motor

volatile uint32_t travelTime = 0;     // the time it takes the sound to comback to the sensor in micoseconds
int16_t distance = 0;                 // distance from de ultrasoic sensor in cm
volatile bool newMesurement = false;  // is true when the interupt is triggerd to indicate a new mesurement
uint8_t controlMode = 0;              // 0 = off, 1 = manuel, 2 = Vvl, 3 = HOME, 4 = balans en 5 = Avl
uint8_t cursorPlace = 0;              // is used to select the parameter that you want to change when in PID controlmode
uint16_t pidChangeDetection = 0;      // is used to see if there are changes in the PID setting
int16_t kp_Vvl = 0;                       // P parameter from the PID voorvleugel
int16_t ki_Vvl = 0;                       // I parameter from the PID voorvleugel
int16_t kd_Vvl = 0;                       // D parameter from the PID voorvleugel
int16_t kp_Avl = 0;                       // P parameter from the PID voorvleugel
int16_t ki_Avl = 0;                       // I parameter from the PID voorvleugel
int16_t kd_Avl = 0;                       // D parameter from the PID voorvleugel
int16_t kp_balans = 0;                       // P parameter from the PID voorvleugel
int16_t ki_balans = 0;                       // I parameter from the PID voorvleugel
int16_t kd_balans = 0;                       // D parameter from the PID voorvleugel
uint8_t setDistance = 20;             // target distance in cm that the PID will try to reach, this value can be changed on de
int8_t setRoll = 0;             // target roll in 10de graden( 1 = 0,1 graden en 10 = 1 graad) that the PID will try to reach, this value can be changed on de
uint8_t setPitch = 0;             // target pitch in 10de graden( 1 = 0,1 graden en 10 = 1 graad) that the PID will try to reach, this value can be changed on de
int16_t pulsen_offset = 0;            // berekende pulsen offset
int16_t leftAcutatorStroke = 150;     // amount of mm the actuator is extened. value is 150 so you dont have to press the button 1000x. the vlalue will only be send when the controllmode is != OFF
int16_t rightAcutatorStroke = 150;    // amount of mm the actuator is extened. value is 150 so you dont have to press the button 1000x. the vlalue will only be send when the controllmode is != OFF
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

// data van CAN
float pitch;
float roll;
uint8_t overcurrent_achter;
int16_t PWM_links;
int16_t PWM_rechts;
int16_t PWM_achter;

int16_t CAN_pulsen_voor = 123;
int16_t CAN_pulsen_offset = 456;
int16_t CAN_pulsen_achter = 789;

uint8_t home_front_foil;
uint8_t home_rear_foi;

#include <LiquidCrystal.h>
LiquidCrystal lcd(RS, E, D4, D5, D6, D7);
//RunningMedian travelTimeMedian = RunningMedian(medianSize);

void setup() {
  Serial.begin(250000);
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
  static uint16_t lastPidChangeDetection = 1;
  if (((newMesurement || (pidChangeDetection != lastPidChangeDetection)) && controlMode == 2)) {
    newMesurement = false;
    computePid_Vvl();
  }

  //================================================================== main loop display data ==========================================================================

  static uint32_t lastRefreshDistanceDisplay = 0;
  if (millis() - lastRefreshDistanceDisplay > refreshDistanceDisplay) {
    lastRefreshDistanceDisplay = millis();
    computeDistance();  // can stay in display data instead off compute date because it is only uesed here
    displayDistance();
  }
  static int16_t lastLeftAcutatorStroke = 0;
  static int16_t lastRightAcutatorStroke = 0;

  if ((leftAcutatorStroke != lastLeftAcutatorStroke) || (rightAcutatorStroke != lastRightAcutatorStroke)) {
    lastLeftAcutatorStroke = leftAcutatorStroke;
    lastRightAcutatorStroke = rightAcutatorStroke;
    displayActuatorStroke();
  }
  if ((pidChangeDetection != lastPidChangeDetection) && controlMode == 2) {  // wanneer de PID ingesteld word
    lastPidChangeDetection = pidChangeDetection;
    pidDisplay();
  }
  static uint8_t lastControlMode = 255;  // use 255 so that it runs at least ones to display the data

  if (controlMode != lastControlMode) {
    lastControlMode = controlMode;
    displayControlMode();
  }

  read_CAN_data();

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

    } else if (canMsg.can_id == 0x32) {
      PWM_links = int16_from_can(canMsg.data[0], canMsg.data[1]);   //byte 0-1 is int16_t PWM links
      PWM_rechts = int16_from_can(canMsg.data[2], canMsg.data[3]);  //byte 0-1 is int16_t PWM rechts

    } else if (canMsg.can_id == 0x33) {
      PWM_achter = int16_from_can(canMsg.data[0], canMsg.data[1]);  //byte 0-1 is int16_t PWM achter
      overcurrent_achter = canMsg.data[2];                          //byte 2 is uint8_t overcurrent achter uint8_t
      Serial.println(PWM_achter);
    }
  }
}

//========================================================================= send_CAN_data ==================================================================

void send_CAN_data() {
  int_to_frame_thrice(CAN_pulsen_voor, CAN_pulsen_offset, CAN_pulsen_achter, 200);
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
  if (cursorPlace == 1) {  // if 1 change the differnce between left and right actuatorStroke parameter
    lcd.setCursor(12, 0);
    lcd.print F((">"));
    lcd.setCursor(12, 1);
    lcd.print F((">"));
  } else {
    lcd.setCursor(12, 0);
    lcd.print F(("L"));
    lcd.setCursor(12, 1);
    lcd.print F(("R"));
  }
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

//===================================================================== computeButtonPress =========================================================================

void computeButtonPress() {
  static int8_t differnce = 0;
  static int prev_enc_1_pulses;
  static int prev_enc_2_pulses;
  differnce = leftAcutatorStroke - rightAcutatorStroke;

  if (enc_1_pulses < prev_enc_1_pulses) {
    controlMode--;
  } else if (enc_1_pulses > prev_enc_1_pulses) {
    controlMode++;
  } 
  prev_enc_1_pulses = enc_1_pulses;
  if (enc_2_pulses < prev_enc_2_pulses) {
    cursorPlace--;
  } else if (enc_2_pulses > prev_enc_2_pulses) {
    cursorPlace++;
  }
  prev_enc_2_pulses = enc_2_pulses;
        if (cursorPlace == 5) {
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
      if (cursorPlace == 5) {
        cursorPlace = 0;
      }
    } else if ((button3 == HIGH) && (buttonStateChange3) && (cursorPlace == 0)) {  // if cursor place is at 0 change setDistance
      setDistance--;
    } else if ((button4 == HIGH) && (buttonStateChange4) && (cursorPlace == 0)) {
      setDistance++;
    } else if ((button3 == HIGH) && (buttonStateChange3) && (cursorPlace == 1)) {  // if cursor place is at 1 change roll setpoint
        setRoll++;
    } else if ((button4 == HIGH) && (buttonStateChange4) && (cursorPlace == 1)) {
        setRoll--;
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
    }
    pidChangeDetection = setDistance + cursorPlace + kp_Vvl + ki_Vvl + kd_Vvl;
  }

  leftAcutatorStroke = constrain(leftAcutatorStroke, 0, 300);
  rightAcutatorStroke = constrain(rightAcutatorStroke, 0, 300);
  cursorPlace = constrain(cursorPlace, 0, 4);
  controlMode = constrain(controlMode, 0, 5);
  setDistance = constrain(setDistance, 20, 99);
  kp_Vvl = constrain(kp_Vvl, 0, 999);
  ki_Vvl = constrain(ki_Vvl, 0, 999);
  kd_Vvl = constrain(kd_Vvl, 0, 999);
}

//======================================================================= computeDistance ==========================================================================

void computeDistance() {
  static int16_t distance_sensor;
  distance_sensor = (travelTime + (0.5 * soundSpeed)) / soundSpeed;  // afstand in cm. because int are rounded down we add 0,5 cm or 29 micoseconds
  static float pitch_rad;                                            // arduino werkt in radians.
  pitch_rad = pitch * M_PI / 180.0;                                  // degees to radians
  distance = distance_sensor - (270 * tan(pitch_rad)) - 40 + 0.5;    // afstand van de onderkant van de boot (-40) tot het water onder de vleugel door rekening te houden met de hoek van de boot(-270*tan(pitch_rad). because int are rounded down we add 0,5
}

//======================================================================= computePid ==========================================================================

void computePid_Vvl() {
  static float error = 0;
  static float diffError = 0;
  static float diffErrorFilter = 0;
  static float oldError = 0;
  static float P = 0;
  static float I = 0;
  static float D = 0;
  static float pidVvlTotal = 0;
  static int16_t pidOffset = 0;
  static uint32_t lastPidTime = 0;
  static uint32_t pidTime = 0;
  static int32_t leftAcutatorPos;
  static int32_t rightAcutatorPos;
  static int16_t pidLoopTime_ms = 0;
  static float pidLoopTime_s = 0;
  static float hoek_voor_vleugel = 0;
  static uint16_t pulsen_liniear;
  static float afstand_liniear;

  pidTime = millis();
  pidLoopTime_ms = pidTime - lastPidTime;
  lastPidTime = pidTime;
  pidLoopTime_s = float(pidLoopTime_ms) / 1000.0;
  error = float(setDistance) - float(distance);
  diffError = error - oldError;
  oldError = error;
  diffErrorFilter = diffErrorFilter * 0.7 + diffError * 0.3;  // filter om te voorkomen dat de D te aggrasief reageert op ruis.

  P = float(kp_Vvl) * error / 100.0;  // delen door 100 om komma te besparen op het display.
  if ((abs(PWM_links) + abs(PWM_rechts)) != 800) {
    I = I + (float(ki_Vvl) * error * pidLoopTime_s / 100.0);
  }
  D = (float(kd_Vvl) * float(diffErrorFilter) / pidLoopTime_s) / 100.0;

  P = constrain(P, -12.0, 12.0);
  I = constrain(I, -12.0, 12.0);
  D = constrain(D, -12.0, 12.0);

  if (ki_Vvl == 0) {
    I = 0.0;
  }
  pidVvlTotal = P + I + D;  // PID wordt berekend in graden
  Serial.println(pidVvlTotal);

  pidVvlTotal = constrain(pidVvlTotal, -12.0, 12.0);

  hoek_voor_vleugel = pidVvlTotal - pitch;
  afstand_liniear = sqrt(43.2 * 43.2 + 13.4 * 13.4 - 2 * 43.2 * 13.4 * cos((hoek_voor_vleugel + 90.0 - 22.49831) * M_PI / 180.0));  // lengte linieare motor in cm is wortel(b^2+c^2 - 2*b*c*cos(hoek vleugel))
  pulsen_liniear = afstand_liniear * pulsen_per_mm * 10;                                                                            // pulsen naar voorvleugel = afstand in cm maal pulsen per cm
  CAN_pulsen_voor = pulsen_liniear;
}

//======================================================================= displayDistance ==========================================================================

void displayDistance() {
  lcd.setCursor(0, 0);  // set curser at distantce place
  int x constrain(distance, 0, 99);
  if (x == 0) {              // check for error
    lcd.print F(("ERROR"));  // print error
  } else {                   // if no error print the distance
    if (x < 10) {
      lcd.print F(("  "));
    } else if (x < 100) {
      lcd.print F((" "));
    }
    lcd.print(x);
    lcd.print F(("cm"));  // print unit cm for distance
  }
}

//======================================================================= displayActuatorStroke ==========================================================================

void displayActuatorStroke() {
  if (controlMode == 2) {
    lcd.setCursor(13, 0);  // set cursor for left acutator distance in mm
  } else {
    lcd.setCursor(10, 0);  // set cursor for left acutator distance in mm
    lcd.print F(("L"));
  }
  if (leftAcutatorStroke < 10) {
    lcd.print F(("  "));
  } else if (leftAcutatorStroke < 100) {
    lcd.print F((" "));
  }
  lcd.print(leftAcutatorStroke);  // print distance right acutator
  lcd.print F(("mm"));            // print unit mm for acutator distance

  if (controlMode == 2) {
    lcd.setCursor(13, 1);  // set cursor for left acutator distance in mm
  } else {
    lcd.setCursor(10, 1);  // set cursor for left acutator distance in mm
    lcd.print F(("R"));
  }
  if (rightAcutatorStroke < 10) {
    lcd.print F(("  "));
  } else if (rightAcutatorStroke < 100) {
    lcd.print F((" "));
  }
  lcd.print(rightAcutatorStroke);  // print distance right acutator
  lcd.print F(("mm"));             // print unit mm for acutator distance
}

//======================================================================= displayControlMode ==========================================================================

void displayControlMode() {
  lcd.setCursor(12, 0);
  if (controlMode == 0) {  // check controlmode. for off, manuel or automatic PID control
    lcd.print F(("OFF"));
  } else if (controlMode == 1) {
    lcd.print F(("MAN"));
  } else if (controlMode == 2) {
    lcd.print F(("V vl"));
  } else if (controlMode == 3) {
    lcd.print F(("Home"));
  } else if (controlMode == 4) {
    lcd.print F(("Bal"));
  } else if (controlMode == 5) {
    lcd.print F(("A vl"));
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
    newMesurement = true;

    //Serial.print(newMesurement);

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
  memcpy(bytes + sizeof(int16_t), &i16_1, sizeof(int16_t));
  memcpy(bytes + sizeof(int16_t) * 2, &i16_3, sizeof(int16_t));
  can_frame ret;
  for (uint8_t i = 0; i < sizeof(int16_t) * 3; i++) {
    ret.data[i] = bytes[i];
  }
  ret.can_id = can_id;
  ret.can_dlc = sizeof(int16_t) * 3;
  return ret;
}
