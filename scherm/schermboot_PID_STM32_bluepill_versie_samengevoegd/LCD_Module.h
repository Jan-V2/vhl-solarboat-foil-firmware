#pragma once

namespace LCD_Module {

//LCD //KLOPT
#define E PC14
#define RS PC15
#define D4 PA0
#define D5 PA1
#define D6 PA2
#define D7 PA3

const uint16_t refreshDistanceDisplay = 399;  // How many milliseconds between display updates

byte smile_happy[8] =

  {
    0b00000,
    0b00000,
    0b01010,
    0b00000,
    0b10001,
    0b01110,
    0b00000,
    0b00000
  };

byte smile_neutraal[8] =

  {
    0b00000,
    0b00000,
    0b01010,
    0b00000,
    0b00000,
    0b01110,
    0b00000,
    0b00000
  };

byte smile_sad[8] =

  {
    0b11111,
    0b11111,
    0b10101,
    0b11111,
    0b11111,
    0b10001,
    0b01110,
    0b11111
  };

// begin setup
void setup_LCD_Module() {
  lcd.createChar(1, smile_happy);
  lcd.createChar(2, smile_neutraal);
  lcd.createChar(3, smile_sad);

  lcd.begin(20, 4);  // Switch on the LCD screen
  lcd.setCursor(2, 0);
  lcd.print F(("VHL-Nordwin"));  // Print these words to my LCD screen
  lcd.setCursor(1, 2);
  lcd.print F(("Zonnebootteam"));
}
// einde setup

}  // namespace CAN_Module
