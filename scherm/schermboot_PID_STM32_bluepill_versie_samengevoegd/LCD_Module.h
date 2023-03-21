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
}  // einde setup

// begin loop
void LCD_Module_loop() {
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
}  // einde loop

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

}  // namespace CAN_Module
