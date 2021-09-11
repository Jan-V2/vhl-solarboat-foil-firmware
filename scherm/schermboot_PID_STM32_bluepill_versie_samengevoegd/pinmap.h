// de cpu is Geen stm32f103RBT6 MAAR STM32F103C8T6 digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

//SPI 1 //KLOPT
/*
CS CAN1 PA4
CS CAN2 PB0
CLK	PA5
MISO	PA6
MOSI	PA7
*/

// LED's //KLOPT
#define LED_BUILTIN PC13
//#define LED_GREEN PC13
//#define LED_ORANGE PC14
//#define LED_RED PC15

//input/buttons // KLOPT
#define BUTTON_4 PB7
#define BUTTON_3 PB6
#define BUTTON_2 PB5
#define BUTTON_1 PB4
#define ENC_1A PB9
#define ENC_1B PB8
#define ENC_2A PB15
#define ENC_2B PA14
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

void encoder1_ISR(){
  if (digitalRead(ENC_1B)){
    enc_1_pulses++;
  }else{
    enc_1_pulses--;
  }
}


void encoder2_ISR(){
  if (digitalRead(ENC_2B)){
    enc_2_pulses++;
  }else{
    enc_2_pulses--;
  }
}


void setup_buttons_and_encoders(){    
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);
    pinMode(BUTTON_3, INPUT_PULLUP);
    pinMode(BUTTON_4, INPUT_PULLUP);
  
    pinMode(ENC_1_BTN, INPUT_PULLUP);  
    pinMode(ENC_2_BTN, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(ENC_1A), encoder1_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC_2A), encoder2_ISR, FALLING);
}
