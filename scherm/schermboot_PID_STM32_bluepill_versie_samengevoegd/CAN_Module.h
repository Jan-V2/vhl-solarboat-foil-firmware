#pragma once

#include "Ultrasonic_Module.h"

#include <mcp2515.h>

namespace CAN_Module {

enum CAN_netwerk {
  telemetry,
  motor
};

struct can_frame canMsg;
MCP2515 mcp2515_telemetry(PB0);  
MCP2515 mcp2515_motor(PA4);

const uint8_t SendCanMotorTime = 10;             // How many milliseconds between sending CAN frames to the CAN_netwerk motor
const uint8_t SendCanTelemetryTime = 10;         // How many milliseconds between sending CAN frames to the CAN_netwerk telemetry
const int16_t SendCanTelemetryTimeStatus = 450;  // iedere seconden word er een berichtje naar de telemetrie verstuurd om te laten weten dat het scherm werkt en de voo/achter vleugel
const int16_t offline_time = 2500;               // als de voor of achtervleugel langer dan 2,5 seconden niks over de can versturen word de online status false. deze info word naar de telemetrie en scherm verstuurd

float P_Vvl;               // tijdelijk voor debuggen moet uit de PID_vleugels.h komen
float I_Vvl;               // tijdelijk voor debuggen moet uit de PID_vleugels.h komen
float D_Vvl;               // tijdelijk voor debuggen moet uit de PID_vleugels.h komen
float P_Avl;               // tijdelijk voor debuggen moet uit de PID_vleugels.h komen
float I_Avl;               // tijdelijk voor debuggen moet uit de PID_vleugels.h komen
float D_Avl;               // tijdelijk voor debuggen moet uit de PID_vleugels.h komen
float P_Balans;            // tijdelijk voor debuggen moet uit de PID_vleugels.h komen
float I_Balans;            // tijdelijk voor debuggen moet uit de PID_vleugels.h komen
float D_Balans;            // tijdelijk voor debuggen moet uit de PID_vleugels.h komen
float pidVvlTotal = 0;     // tijdelijk voor debuggen moet uit de PID_vleugels.h komen
float pidAvlTotal = 0;     // tijdelijk voor debuggen moet uit de PID_vleugels.h komen
float pidBalansTotal = 0;  // tijdelijk voor debuggen moet uit de PID_vleugels.h komen

bool pid_actief = false;   // tijdelijk voor debuggen moet uit de Globals.h komen

int16_t P_Vvl_telemetry;
int16_t I_Vvl_telemetry;
int16_t D_Vvl_telemetry;
int16_t P_Avl_telemetry;
int16_t I_Avl_telemetry;
int16_t D_Avl_telemetry;
int8_t P_Balans_telemetry;
int8_t I_Balans_telemetry;
int8_t D_Balans_telemetry;
int16_t pidVvlTotal_telemetry;
int16_t pidAvlTotal_telemetry;
int8_t pidBalansTotal_telemetry;
int8_t distance_telemetry;
bool PID_debug_telemetry;
bool status_Vvl = false;
bool status_Avl = false;
bool status_gryo = false;
bool status_gashendel = false;
bool status_temp = false;
bool status_telemetrie_verbinding_server = false;
bool status_telemetrie_verbinding_CAN = false;

uint32_t last_Vvl_online_millis;
uint32_t last_Avl_online_millis;
uint32_t last_gyro_online_millis;

// data van CAN
float pitch;
float roll;
int16_t amps_achter_vleugel;
int16_t PWM_links;
int16_t PWM_rechts;
int16_t PWM_achter;

int16_t CAN_pulsen_voor = 0;
int16_t CAN_pulsen_offset = 0;
int16_t CAN_pulsen_achter = 0;

bool home_front_foil;
bool home_rear_foil;
int16_t has_homed_voor_vleugel;
int16_t has_homed_achter_vleugel;

// Functions have to be decalred before they can be used in another function this is called a "forward declaration"
void read_CAN_data();
void send_CAN_data_motor();
void send_CAN_data_telemetry();

float float_from_can(uint8_t start_idx);
int16_t int16_from_can(uint8_t b1, uint8_t b2);

can_frame int_to_frame_thrice(int16_t i16_1, int16_t i16_2, int16_t i16_3, int16_t i16_4, uint16_t can_id, CAN_netwerk CAN_controller);
can_frame int8_t_to_frame(int8_t i8_1, int8_t i8_2, int8_t i8_3, int8_t i8_4, int8_t i8_5, int8_t i8_6, int8_t i8_7, int8_t i8_8, uint16_t can_id, CAN_netwerk CAN_controller);
can_frame bool_to_frame(bool b, uint16_t can_id, CAN_netwerk CAN_controller);

// ===== CAN Module setup =====
void setup_CAN_Module() {
  mcp2515_telemetry.reset();
  mcp2515_telemetry.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515_telemetry.setNormalMode();

  mcp2515_motor.reset();
  mcp2515_motor.setBitrate(CAN_125KBPS);
  mcp2515_motor.setNormalMode();
}

// ===== start CAN loop =====
void CAN_Module_loop() {

  read_CAN_data();

  static uint32_t lastSendCanMotor = 0;
  if (millis() - lastSendCanMotor > SendCanMotorTime) {
    lastSendCanMotor = millis();
    send_CAN_data_motor();
  }

  static uint32_t lastSendCanTelemetry = 0;
  if (millis() - lastSendCanTelemetry > SendCanTelemetryTime) {
    lastSendCanTelemetry = millis();
    send_CAN_data_telemetry();
  }

  static uint32_t lastSendCanTelemetryStatus = 0;
  //  static uint32_t last_Vvl_online_millis = 0;
  //  static uint32_t last_Avl_online_millis = 0;

  if (millis() - lastSendCanTelemetryStatus > SendCanTelemetryTimeStatus) {
    lastSendCanTelemetryStatus = millis();
    if (millis() - last_Vvl_online_millis > offline_time) {
      status_Vvl = false;
    } else {
      status_Vvl = true;
    }
    if (millis() - last_Avl_online_millis > offline_time) {
      status_Avl = false;
    } else {
      status_Avl = true;
    }
    if (millis() - last_gyro_online_millis > offline_time) {
      status_gryo = false;
    } else {
      status_gryo = true;
    }

    int8_t_to_frame(status_Vvl, status_Avl, 0, 0, 0, 0, 0, 0, 54, telemetry);
  }
}
// ===== einde CAN loop =====

//========================================================================= send_CAN_data_telemetry ==================================================================

void send_CAN_data_telemetry() {
  P_Vvl_telemetry = constrain(P_Vvl, -12.0, 12.0) * 100.0;
  I_Vvl_telemetry = constrain(I_Vvl, -12.0, 12.0) * 100.0;
  D_Vvl_telemetry = constrain(D_Vvl, -12.0, 12.0) * 100.0;
  pidVvlTotal_telemetry = constrain(pidVvlTotal, -12.0, 12.0) * 100.0;
  P_Avl_telemetry = constrain(P_Avl, -12.0, 12.0) * 100.0;
  I_Avl_telemetry = constrain(I_Avl, -12.0, 12.0) * 100.0;
  D_Avl_telemetry = constrain(D_Avl, -12.0, 12.0) * 100.0;
  pidAvlTotal_telemetry = constrain(pidAvlTotal, -12.0, 12.0) * 100.0;
  P_Balans_telemetry = constrain(P_Balans, -5.0, 5.0) * 20.0;
  I_Balans_telemetry = constrain(I_Balans, -5.0, 5.0) * 20.0;
  D_Balans_telemetry = constrain(D_Balans, -5.0, 5.0) * 20.0;
  pidBalansTotal_telemetry = constrain(pidBalansTotal, -5.0, 5.0) * 20.0;
  distance_telemetry = constrain(Ultrasonic_Module::distance, -127, 127);

  int_to_frame_thrice(P_Vvl_telemetry, I_Vvl_telemetry, D_Vvl_telemetry, pidVvlTotal_telemetry, 51, telemetry);
  int_to_frame_thrice(P_Avl_telemetry, I_Avl_telemetry, D_Avl_telemetry, pidAvlTotal_telemetry, 52, telemetry);
  int8_t_to_frame(P_Balans_telemetry, I_Balans_telemetry, D_Balans_telemetry, pidBalansTotal_telemetry, distance_telemetry, PID_debug_telemetry, 0, 0, 53, telemetry);
}

//========================================================================= send_CAN_data_motor ==================================================================

void send_CAN_data_motor() {
  if (!home_front_foil && !home_rear_foil && pid_actief) {
    int_to_frame_thrice(CAN_pulsen_voor, CAN_pulsen_offset, CAN_pulsen_achter, 0, 200, motor);
    Serial.println(CAN_pulsen_achter);
  }
  if (home_front_foil) {
    bool_to_frame(home_front_foil, 301, motor);
  }
  if (home_rear_foil) {
    bool_to_frame(home_rear_foil, 300, motor);
  }
}

//======================================================================== read_CAN_data ======================================================================

void read_CAN_data() {
  if (mcp2515_motor.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    //  Serial.print("CAN ID: ");
    //  Serial.println(canMsg.can_id, DEC);

    if (canMsg.can_id == 0x64) {
      pitch = float_from_can(0);                // byte 0-3 is float pitch
      roll = float_from_can(4);                 // byte 4-7 is float roll
      Ultrasonic_Module::newMesurement = true;  // er is een nieuwe meting voor de PID compute
      last_gyro_online_millis = millis();

      can_frame ret;
      for (uint8_t i = 0; i < sizeof(float) * 2; i++) {
        ret.data[i] = canMsg.data[i];
      }
      ret.can_id = 50;
      ret.can_dlc = sizeof(float) * 2;
      mcp2515_telemetry.sendMessage(&ret);  // verstuur pitch en roll door naar de telemetry
    }
  } else if (canMsg.can_id == 0x34) {                             // CAN ID 52
    PWM_links = int16_from_can(canMsg.data[0], canMsg.data[1]);   // byte 0-1 is int16_t amps links
    PWM_rechts = int16_from_can(canMsg.data[2], canMsg.data[3]);  // byte 0-1 is int16_t amps rechts
    last_Vvl_online_millis = millis();

  } else if (canMsg.can_id == 0x33) {                             // CAN ID 51
    PWM_links = int16_from_can(canMsg.data[0], canMsg.data[1]);   // byte 0-1 is int16_t PWM links
    PWM_rechts = int16_from_can(canMsg.data[2], canMsg.data[3]);  // byte 0-1 is int16_t PWM rechts
    has_homed_voor_vleugel = int16_from_can(canMsg.data[4], canMsg.data[5]);
    last_Vvl_online_millis = millis();

  } else if (canMsg.can_id == 0x32) {                                           // CAN ID 50
    PWM_achter = int16_from_can(canMsg.data[0], canMsg.data[1]);                // byte 0-1 is int16_t PWM achter
    amps_achter_vleugel = int16_from_can(canMsg.data[2], canMsg.data[3]);       // byte 2-3 is uint8_t overcurrent achter uint8_t
    has_homed_achter_vleugel = int16_from_can(canMsg.data[4], canMsg.data[5]);  // byte 3-4 is bool has homed achtervleugel
    last_Avl_online_millis = millis();
  }
}

// convert can frame to variable

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

can_frame int_to_frame_thrice(int16_t i16_1, int16_t i16_2, int16_t i16_3, int16_t i16_4, uint16_t can_id, CAN_netwerk CAN_controller) {
  byte bytes[sizeof(int16_t) * 4];
  memcpy(bytes, &i16_1, sizeof(int16_t));
  memcpy(bytes + sizeof(int16_t), &i16_2, sizeof(int16_t));
  memcpy(bytes + sizeof(int16_t) * 2, &i16_3, sizeof(int16_t));
  memcpy(bytes + sizeof(int16_t) * 3, &i16_4, sizeof(int16_t));
  can_frame ret;
  for (uint8_t i = 0; i < sizeof(int16_t) * 4; i++) {
    ret.data[i] = bytes[i];
  }
  ret.can_id = can_id;
  ret.can_dlc = sizeof(int16_t) * 4;
  if (CAN_controller == motor) {
    mcp2515_motor.sendMessage(&ret);
  } else if (CAN_controller == telemetry) {
    mcp2515_telemetry.sendMessage(&ret);
  }
  return ret;
}

can_frame int8_t_to_frame(int8_t i8_1, int8_t i8_2, int8_t i8_3, int8_t i8_4, int8_t i8_5, int8_t i8_6, int8_t i8_7, int8_t i8_8, uint16_t can_id, CAN_netwerk CAN_controller) {
  byte bytes[sizeof(int8_t) * 8];
  memcpy(bytes, &i8_1, sizeof(int8_t));
  memcpy(bytes + sizeof(int8_t), &i8_2, sizeof(int8_t));
  memcpy(bytes + sizeof(int8_t) * 2, &i8_3, sizeof(int8_t));
  memcpy(bytes + sizeof(int8_t) * 3, &i8_4, sizeof(int8_t));
  memcpy(bytes + sizeof(int8_t) * 4, &i8_5, sizeof(int8_t));
  memcpy(bytes + sizeof(int8_t) * 5, &i8_6, sizeof(int8_t));
  memcpy(bytes + sizeof(int8_t) * 6, &i8_7, sizeof(int8_t));
  memcpy(bytes + sizeof(int8_t) * 7, &i8_8, sizeof(int8_t));
  can_frame ret;
  for (uint8_t i = 0; i < sizeof(int8_t) * 8; i++) {
    ret.data[i] = bytes[i];
  }
  ret.can_id = can_id;
  ret.can_dlc = sizeof(int8_t) * 8;
  if (CAN_controller == motor) {
    mcp2515_motor.sendMessage(&ret);
  } else if (CAN_controller == telemetry) {
    mcp2515_telemetry.sendMessage(&ret);
  }
  return ret;
}

can_frame bool_to_frame(bool b, uint16_t can_id, CAN_netwerk CAN_controller) {
  byte bytes[sizeof(bool)];
  memcpy(bytes, &b, sizeof(bool));
  can_frame ret;
  for (uint8_t i = 0; i < sizeof(bool); i++) {
    ret.data[i] = bytes[i];
  }
  ret.can_id = can_id;
  ret.can_dlc = sizeof(bool);
  if (CAN_controller == motor) {
    mcp2515_motor.sendMessage(&ret);
  }
  return ret;
}

}  // namespace CAN_Module