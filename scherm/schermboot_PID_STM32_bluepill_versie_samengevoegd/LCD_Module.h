#pragma once

#include <LiquidCrystal.h>


namespace LCD_Module {

//LCD //KLOPT
const uint8_t E = PC14;
const uint8_t RS = PC15;
const uint8_t D4 = PA0;
const uint8_t D5 = PA1;
const uint8_t D6 = PA2;
const uint8_t D7 = PA3;

LiquidCrystal lcd(RS, E, D4, D5, D6, D7);

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
} // einde setup

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

}  // namespace CAN_Module
