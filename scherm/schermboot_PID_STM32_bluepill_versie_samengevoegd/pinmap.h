// de cpu is een stm32f103RBT6  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

// LED's
#define LED_BUILTIN PC13
#define LED_GREEN PC13
#define LED_ORANGE PC14
#define LED_RED PC15

//input/buttons
#define BUTTON_4 PC3
#define BUTTON_3 PA0
#define BUTTON_2 PA1
#define BUTTON_1 PA2
#define ENC_1A PC5
#define ENC_1B PC4
#define ENC_2A PC2
#define ENC_2B PC1
#define ENC_1_BTN PB0
#define ENC_2_BTN PC0

//ultrasound sensor
#define trig_1 PB3
#define echo_1 PB4
#define trig_2 PB5
#define echo_2 PB6

//LCD 
#define E  PB15
#define RS PB14
#define D4 PC6
#define D5 PC7
#define D6 PC8
#define D7 PC9

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
