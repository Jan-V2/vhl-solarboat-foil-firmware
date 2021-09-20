#pragma once
#ifndef DualVNH5019MotorShield_h
#define DualVNH5019MotorShield_h


 #ifdef __cplusplus
 #define EXTERNC extern "C"
 #else
 #define EXTERNC
 #endif

#ifdef __cplusplus
extern "C" {
#endif
	#include "main.h"
#ifdef __cplusplus
}



const int PWM_COUNTER_MAX = 4000;
const float PWM_MULT_FACTOR = (float)PWM_COUNTER_MAX / 400.0;



struct Gpio_Pin{
	uint16_t pin;
	GPIO_TypeDef* port;
};


class DualVNH5019MotorShield
{
  public:
    // CONSTRUCTORS
    // Default pin selection.
    DualVNH5019MotorShield();
    // User-defined pin selection.
    DualVNH5019MotorShield(unsigned char INA1,
                           unsigned char INB1,
                           unsigned char PWM1,
                           unsigned char EN1DIAG1,
                           unsigned char CS1,
                           unsigned char INA2,
                           unsigned char INB2,
                           unsigned char PWM2,
                           unsigned char EN2DIAG2,
                           unsigned char CS2);

    // PUBLIC METHODS
    void init(); // Initialize TIMER 1, set the PWM to 20kHZ.
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
    void setM1Brake(int brake); // Brake M1.
    void setM2Brake(int brake); // Brake M2.
    void setBrakes(int m1Brake, int m2Brake); // Brake both M1 and M2.
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1.
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
    unsigned char getM1Fault(); // Get fault reading from M1.
    unsigned char getM2Fault(); // Get fault reading from M2.

  private:
    ADC_HandleTypeDef _MOTOR_ADC;
    Gpio_Pin _IN1A;
    Gpio_Pin _IN1B;
    Gpio_Pin _IN2A;
    Gpio_Pin _IN2B;
    volatile uint32_t* _PWM1;
    volatile uint32_t* _PWM2;
    unsigned char _CURRENT_SENSE_CHANNEL_1;
    unsigned char _CURRENT_SENSE_CHANNEL_2;
    Gpio_Pin _FAULT_AND_ENABLE_2;
    Gpio_Pin _FAULT_AND_ENABLE_1;

};
#endif
#endif //DualVNH5019MotorShield_h
