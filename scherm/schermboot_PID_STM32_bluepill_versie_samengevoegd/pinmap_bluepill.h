// de cpu is Geen stm32f103RBT6 MAAR STM32F103C8T6 digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
#pragma once



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
