#pragma once

#include <LiquidCrystal.h>

#include "Globals.h"

//using namespace Globals;

namespace LCD_Module {

// Pin assignment for LCD_Module
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

// Functions have to be decalred before they can be used in another function this is called a "forward declaration"
void setup_LCD_Module();
void LCD_Module_loop();
void pidDisplay();
void displayData();
void blink_cursor();
void displayControlMode();
void OFF();

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
      lcd.print(PID_Berekeningen::kp_Vvl);
      if (PID_Berekeningen::kp_Vvl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kp_Vvl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(0, 1);
      lcd.print F(("P"));
      lcd.print(PID_Berekeningen::kp_Vvl);
      if (PID_Berekeningen::kp_Vvl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kp_Vvl < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 3) {  // if 3 change the I from the PID parameter
      lcd.setCursor(4, 1);
      lcd.print F((">"));
      lcd.print(PID_Berekeningen::ki_Vvl);
      if (PID_Berekeningen::ki_Vvl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::ki_Vvl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(4, 1);
      lcd.print F(("I"));
      lcd.print(PID_Berekeningen::ki_Vvl);
      if (PID_Berekeningen::ki_Vvl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::ki_Vvl < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 4) {  // if 4 change the D from the PID parameter
      lcd.setCursor(8, 1);
      lcd.print F((">"));
      lcd.print(PID_Berekeningen::kd_Vvl);
      if (PID_Berekeningen::kd_Vvl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kd_Vvl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(8, 1);
      lcd.print F(("D"));
      lcd.print(PID_Berekeningen::kd_Vvl);
      if (PID_Berekeningen::kd_Vvl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kd_Vvl < 100) {
        lcd.print F((" "));
      }
    }
  }
  if (menu == Menu::BALANS_VOORVLEUGEL) {  // balans
    if (cursorPlace == 2) {                // if 2 change the P from the PID parameter
      lcd.setCursor(0, 1);
      lcd.print F((">"));
      lcd.print(PID_Berekeningen::kp_balans);
      if (PID_Berekeningen::kp_balans < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kp_balans < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(0, 1);
      lcd.print F(("P"));
      lcd.print(PID_Berekeningen::kp_balans);
      if (PID_Berekeningen::kp_balans < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kp_balans < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 3) {  // if 3 change the I from the PID parameter
      lcd.setCursor(4, 1);
      lcd.print F((">"));
      lcd.print(PID_Berekeningen::ki_balans);
      if (PID_Berekeningen::ki_balans < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::ki_balans < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(4, 1);
      lcd.print F(("I"));
      lcd.print(PID_Berekeningen::ki_balans);
      if (PID_Berekeningen::ki_balans < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::ki_balans < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 4) {  // if 4 change the D from the PID parameter
      lcd.setCursor(8, 1);
      lcd.print F((">"));
      lcd.print(PID_Berekeningen::kd_balans);
      if (PID_Berekeningen::kd_balans < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kd_balans < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(8, 1);
      lcd.print F(("D"));
      lcd.print(PID_Berekeningen::kd_balans);
      if (PID_Berekeningen::kd_balans < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kd_balans < 100) {
        lcd.print F((" "));
      }
    }
  }
  if (menu == Menu::ACHTERVLEUGEL) {  // achtervleugel
    if (cursorPlace == 2) {           // if 2 change the P from the PID parameter
      lcd.setCursor(0, 1);
      lcd.print F((">"));
      lcd.print(PID_Berekeningen::kp_Avl);
      if (PID_Berekeningen::kp_Avl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kp_Avl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(0, 1);
      lcd.print F(("P"));
      lcd.print(PID_Berekeningen::kp_Avl);
      if (PID_Berekeningen::kp_Avl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kp_Avl < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 3) {  // if 3 change the I from the PID parameter
      lcd.setCursor(4, 1);
      lcd.print F((">"));
      lcd.print(PID_Berekeningen::ki_Avl);
      if (PID_Berekeningen::ki_Avl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::ki_Avl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(4, 1);
      lcd.print F(("I"));
      lcd.print(PID_Berekeningen::ki_Avl);
      if (PID_Berekeningen::ki_Avl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::ki_Avl < 100) {
        lcd.print F((" "));
      }
    }
    if (cursorPlace == 4) {  // if 4 change the D from the PID parameter
      lcd.setCursor(8, 1);
      lcd.print F((">"));
      lcd.print(PID_Berekeningen::kd_Avl);
      if (PID_Berekeningen::kd_Avl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kd_Avl < 100) {
        lcd.print F((" "));
      }
    } else {
      lcd.setCursor(8, 1);
      lcd.print F(("D"));
      lcd.print(PID_Berekeningen::kd_Avl);
      if (PID_Berekeningen::kd_Avl < 10) {
        lcd.print F(("  "));
      } else if (PID_Berekeningen::kd_Avl < 100) {
        lcd.print F((" "));
      }
    }
  }
}  // einde PID_display

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
      if (PID_Berekeningen::P_Vvl >= 0) {
        lcd.print F((" "));
        if (PID_Berekeningen::P_Vvl < 1) {
          lcd.print(" ");
        }
      } else if (PID_Berekeningen::P_Vvl > -1.0) {
        lcd.print(" ");
      }
      lcd.print(PID_Berekeningen::P_Vvl * 10.0, 0);

      lcd.setCursor(4, 2);  // print I_Vvl
      lcd.print(" I");
      if (PID_Berekeningen::I_Vvl >= 0) {
        lcd.print F((" "));
        if (PID_Berekeningen::I_Vvl < 1) {
          lcd.print(" ");
        }
      } else if (PID_Berekeningen::I_Vvl > -1.0) {
        lcd.print(" ");
      }
      lcd.print(PID_Berekeningen::I_Vvl * 10.0, 0);

      lcd.setCursor(9, 2);  // print D_Vvl
      lcd.print(" D");
      if (PID_Berekeningen::D_Vvl >= 0.0) {
        lcd.print F((" "));
        if (PID_Berekeningen::D_Vvl < 1.0) {
          lcd.print(" ");
        }
      } else if (PID_Berekeningen::D_Vvl > -1.0) {
        lcd.print(" ");
      }
      lcd.print(PID_Berekeningen::D_Vvl * 10.0, 0);

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

    case Globels::Menu::ACHTERVLEUGEL:  // controlMode achtervleugel
      lcd.setCursor(0, 2);     // print P_Avl
      lcd.print("P");
      if (PID_Berekeningen::P_Avl >= 0) {
        lcd.print F((" "));
        if (PID_Berekeningen::P_Avl < 1) {
          lcd.print(" ");
        }
      } else if (PID_Berekeningen::P_Avl > -1.0) {
        lcd.print(" ");
      }

      lcd.setCursor(4, 2);  // print I_Avl
      lcd.print(" I");
      if (PID_Berekeningen::I_Avl >= 0) {
        lcd.print F((" "));
        if (PID_Berekeningen::I_Avl < 1) {
          lcd.print(" ");
        }
      } else if (PID_Berekeningen::I_Avl > -1.0) {
        lcd.print(" ");
      }
      lcd.print(PID_Berekeningen::I_Avl * 10.0, 0);

      lcd.setCursor(9, 2);  // print D_Avl
      lcd.print(" D");
      if (PID_Berekeningen::D_Avl >= 0.0) {
        lcd.print F((" "));
        if (PID_Berekeningen::D_Avl < 1.0) {
          lcd.print(" ");
        }
      } else if (PID_Berekeningen::D_Avl > -1.0) {
        lcd.print(" ");
      }
      lcd.print(PID_Berekeningen::D_Avl * 10.0, 0);

      lcd.setCursor(0, 3);  // print PID_Avl
      lcd.print("PID");
      if ((PID_Berekeningen::pidAvlTotal < 10.0) && (PID_Berekeningen::pidAvlTotal >= 0.0)) {
        lcd.print(" ");
        if (PID_Berekeningen::pidAvlTotal < 1) {
          lcd.print(" ");
        }
      }
      if (PID_Berekeningen::pidAvlTotal < 0.0 && PID_Berekeningen::pidAvlTotal > -1.0) {
        lcd.print(" ");
      }
      lcd.print(PID_Berekeningen::pidAvlTotal * 10.0, 0);
      lcd.print(' ');
      break;

    case Globels::Menu::BALANS_VOORVLEUGEL:  // controlMode balansvleugel
      lcd.setCursor(0, 2);          // print P_Vvl
      lcd.print("P");
      if (PID_Berekeningen::P_Balans >= 0) {
        lcd.print F((" "));
        if (PID_Berekeningen::P_Balans < 1) {
          lcd.print(" ");
        }
      } else if (PID_Berekeningen::P_Balans > -1.0) {
        lcd.print(" ");
      }
      lcd.print(PID_Berekeningen::P_Balans * 10.0, 0);

      lcd.setCursor(4, 2);  // print I_Vvl
      lcd.print(" I");
      if (I_Balans >= 0) {
        lcd.print F((" "));
        if (I_Balans < 1) {
          lcd.print(" ");
        }
      } else if (PID_Berekeningen::I_Balans > -1.0) {
        lcd.print(" ");
      }
      lcd.print(PID_Berekeningen::I_Balans * 10.0, 0);

      lcd.setCursor(9, 2);  // print D_Balans
      lcd.print(" D");
      if (PID_Berekeningen::D_Balans
          >= 0.0) {
        lcd.print F((" "));
        if (PID_Berekeningen::D_Balans < 1.0) {
          lcd.print(" ");
        }
      } else if (PID_Berekeningen::D_Balans > -1.0) {
        lcd.print(" ");
      }
      lcd.print(PID_Berekeningen::D_Balans * 10.0, 0);

      lcd.setCursor(0, 3);  // print PID_Vvl
      lcd.print("PID");
      if ((PID_Berekeningen::pidBalansTotal < 10.0) && (PID_Berekeningen::pidBalansTotal >= 0.0)) {
        lcd.print(" ");
        if (PID_Berekeningen::pidBalansTotal < 1) {
          lcd.print(" ");
        }
      }
      if (PID_Berekeningen::pidBalansTotal < 0.0 && PID_Berekeningen::pidBalansTotal > -1.0) {
        lcd.print(" ");
      }
      lcd.print(PID_Berekeningen::pidBalansTotal * 10.0, 0);
      lcd.print(' ');
      break;

    case Globels::Menu::OFF:
      break;

    case Globels::Menu::STARTUP:
      break;
  }
}  // einde displayData

// begin blink_cursor
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
}  // einde blink_cursor

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

}  // namespace CAN_Module
