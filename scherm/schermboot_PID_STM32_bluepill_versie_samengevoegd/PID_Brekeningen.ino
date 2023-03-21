#pragma once

#include "LCD_Module.h"
#include "CAN_Module.h"
#include "Ultrasonic_Module.h"
#include "Buttons.h"
#include "Globals.h"

namespace PID_Berekeningen {

int16_t kp_Vvl = 0;               // P parameter from the PID voorvleugel
int16_t ki_Vvl = 0;               // I parameter from the PID voorvleugel
int16_t kd_Vvl = 0;               // D parameter from the PID voorvleugel
int16_t kp_Avl = 0;               // P parameter from the PID voorvleugel
int16_t ki_Avl = 0;               // I parameter from the PID voorvleugel
int16_t kd_Avl = 0;               // D parameter from the PID voorvleugel
int16_t kp_balans = 0;            // P parameter from the PID voorvleugel
int16_t ki_balans = 0;            // I parameter from the PID voorvleugel
int16_t kd_balans = 0;            // D parameter from the PID voorvleugel
uint16_t pidChangeDetection = 0;  // is used to see if there are changes in the PID setting

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

uint8_t setDistance = 10;   // target distance in cm that the PID will try to reach, this value can be changed on de
int8_t setRoll = 0;         // target roll in 10de graden( 1 = 0,1 graden en 10 = 1 graad) that the PID will try to reach, this value can be changed on de
int16_t setPitch = 0;       // target pitch in 10de graden( 1 = 0,1 graden en 10 = 1 graad) that the PID will try to reach, this value can be changed on de
int16_t pulsen_offset = 0;  // berekende pulsen offset

const uint16_t PID_compute_time = 250;                               // How many milliseconds between PID compute.
const uint16_t maxPulseEncoder = 11487;                              // the maximum amount of pulses for the front foil motor encoder
const uint16_t maxAfstandEncoder = 200;                              // de afstand in mm die de voor linieare motor kan uit schuiven
const uint16_t pulsen_per_mm = maxPulseEncoder / maxAfstandEncoder;  // pulsen per mm van de linieare motor
const int16_t minDistance = 5;                                       // als de boot onder de minimale hoogte komt dan wordt de hoek van de vleugel aggresiever.
const int16_t maxDistance = 30;                                      // als de boot boven de maximale hoogte komt dan wordt de hoek van de vleugel minder aggresief.

bool pid_actief = false;  // PID staat uit wanneer false. kan aangepast worden in OFF controlmode 0

}  // namespace PID_Berekeningen
