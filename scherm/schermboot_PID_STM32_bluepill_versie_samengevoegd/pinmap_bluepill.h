// de cpu is Geen stm32f103RBT6 MAAR STM32F103C8T6 digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

//SPI 1 //KLOPT
/*
  CS CAN1 PA4 motor
  CS CAN2 PB0 telemety
  CLK	PA5
  MISO	PA6
  MOSI	PA7
*/

//input/buttons // KLOPT
#define BUTTON_4 PB7
#define BUTTON_3 PB6
#define BUTTON_2 PB5
#define BUTTON_1 PB4
#define ENC_1A PB9
#define ENC_1B PB8
#define ENC_2A PB15
#define ENC_2B PB14
#define ENC_1_BTN PA8
#define ENC_2_BTN PB13

//ultrasound sensor // KLOPT
#define trig_1 PB3
#define echo_1 PA15
#define trig_2 PA12
#define echo_2 PA11

//LCD //KLOPT
#define E  PC14
#define RS PC15
#define D4 PA0
#define D5 PA1
#define D6 PA2
#define D7 PA3

int enc_1_pulses = 0;
int enc_2_pulses = 0;

// Flag from interrupt routine (moved=true)
///volatile bool rotaryEncoder = false; // youtube

// Interrupt routine just sets a flag when rotation is detected
//void rotary() { // youtube
//  rotaryEncoder = true; // youtube
// }

void encoder1_ISR() {
  Serial.println("Rotary 1");
  if (digitalRead(ENC_1B)) {
    enc_1_pulses++;
  } else {
    enc_1_pulses--;
  }
}

void encoder2_ISR() {
  Serial.println("Rotary 2");
  if (digitalRead(ENC_2B)) {
    enc_2_pulses++;
  } else {
    enc_2_pulses--;
  }
}

void setup_buttons_and_encoders() {

  // The module already has pullup resistors on board
  // pinMode(ENC_1A, INPUT_PULLUP); // youtube
  // pinMode(ENC_1B, INPUT_PULLUP); // youtube


  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);
  pinMode(BUTTON_4, INPUT_PULLUP);

  pinMode(ENC_1_BTN, INPUT_PULLUP);
  pinMode(ENC_2_BTN, INPUT_PULLUP);

  // We need to monitor both pins, rising and falling for all states
  // attachInterrupt(digitalPinToInterrupt(ENC_1A), rotary, CHANGE); // youtube
  // attachInterrupt(digitalPinToInterrupt(ENC_1B), rotary, CHANGE); // youtube
  attachInterrupt(digitalPinToInterrupt(ENC_2A), encoder2_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_1A), encoder1_ISR, FALLING);
}

// ======================================= code van youtube =============================
/*
  // Rotary encoder has moved (interrupt tells us) but what happened?
  // See https://www.pinteric.com/rotary.html
  int8_t checkRotaryEncoder() {
  // Reset the flag that brought us here (from ISR)
  rotaryEncoder = false;

  static uint8_t lrmem = 3;
  static int lrsum = 0;
  static int8_t TRANS[] = {0, -1, 1, 14, 1, 0, 14, -1, -1, 14, 0, 1, 14, 1, -1, 0};

  // Read BOTH pin states to deterimine validity of rotation (ie not just switch bounce)
  int8_t l = digitalRead(ENC_1A);
  int8_t r = digitalRead(ENC_2B);

  // Move previous value 2 bits to the left and add in our new values
  lrmem = ((lrmem & 0x03) << 2) + 2 * l + r;

  // Convert the bit pattern to a movement indicator (14 = impossible, ie switch bounce)
  lrsum += TRANS[lrmem];

  // encoder not in the neutral (detent) state 
// if (lrsum % 4 != 0) {
   return 0;
  }
  // encoder in the neutral state - clockwise rotation
// if (lrsum == 4) {
   lrsum = 0;
   return 1;
  }
  // encoder in the neutral state - anti-clockwise rotation
// if (lrsum == -4) {
   lrsum = 0;
   return -1;
  }
  // An impossible rotation has been detected - ignore the movement
  lrsum = 0;
  return 0;
  }
*/
// ====================================== einde code youtube =============================
