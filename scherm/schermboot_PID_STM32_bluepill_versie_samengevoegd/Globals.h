#pragma once

#include <Buttons.h>
#include <LCD_Module.h>

using namespace Buttons;

namespace Globals {

enum class Menu : uint8_t {
  OFF,
  VOORVLEUGEL,
  ACHTERVLEUGEL,
  BALANS_VOORVLEUGEL,
  DEBUG,
  STARTUP
};

Menu menu;

const float pi = 3.14159265359;


void computeButtonPress() {
  if (button_encoder_2 && buttonStateChange_enc_2) {
    switch ( menu) {

      case  Menu::OFF:
         menu =  Menu::VOORVLEUGEL;
        break;

      case  Menu::VOORVLEUGEL:
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

// begin startupMenu
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
} // einde startupMenu

} // namespace Globals