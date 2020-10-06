#include "DualVNH5019MotorShield.h"
#include "types.h"

#define ENC_PIN_1 11
#define ENC_PIN_2 13

#define us_in_second 1000000
#define systick_per_sec 1000

// PWM + means backwards, and - means forwards

// todo change pwm frequency

DualVNH5019MotorShield motor_shield;

volatile int encoder_pulses = 0;

static volatile int encoder_pulses_prev = encoder_pulses;
static volatile u32 systick_count = 0;
static volatile float speed_pulses_per_sec = 0.0;

//TIM_TypeDef *tim_enc = TIM2;
//TIM_TypeDef *tim_speed = TIM2;

static struct {
    int len = 5;
    float prev_vals[5] = {0,0,0,0,0};
    int idx = 0;
    int pref_pulses = 0;
}speed_calc_data;


void encoder_ISR(){
    if (digitalRead(ENC_PIN_2)){
        encoder_pulses--;
    }else{
        encoder_pulses++;
    }
}


void calc_speed() {

        // perform speed calculation
        if (speed_calc_data.idx++ == speed_calc_data.len){
            speed_calc_data.idx = 0;
        }
        speed_calc_data.prev_vals[speed_calc_data.idx] = (float)(encoder_pulses - speed_calc_data.pref_pulses);
        float acc = 0.0;
        for (int i = 0; i < speed_calc_data.len; i++) {
            acc += speed_calc_data.prev_vals[i];
        }
        
        speed_pulses_per_sec = (acc / (float)speed_calc_data.len) * 10.0;
        
}

void rollover_callback(){}


void setup() {
    Serial.begin(115200);
    
    // configure interrupts and timers 
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_1), encoder_ISR, FALLING);
    
    TIM_TypeDef *tim_enc = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(ENC_PIN_1), PinMap_PWM);
    uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(ENC_PIN_1), PinMap_PWM));
   
/*   
    HardwareTimer *tim_enc_isr = new HardwareTimer(tim_enc);
    tim_enc_isr->setInterruptPriority(1,1);
    tim_enc_isr->setMode(channel, TIMER_INPUT_CAPTURE_FALLING, ENC_PIN_1);
    tim_enc_isr->setOverflow(0x10000);
    tim_enc_isr->setPrescaleFactor(1);
    tim_enc_isr->attachInterrupt(channel, encoder_ISR);
    tim_enc_isr->attachInterrupt(rollover_callback);
*/
    HardwareTimer *tim_speed_calc = new HardwareTimer(TIM1);
    tim_speed_calc->setInterruptPriority(2,2);
    tim_speed_calc->setOverflow(1, HERTZ_FORMAT);
    //tim_speed_calc->attachInterrupt(std::bind(calc_speed, &encoder_pulses));
    tim_speed_calc->attachInterrupt(calc_speed);
    
    
    tim_speed_calc->resume();
    
    //tim_enc_isr->resume();
  
    motor_shield.init();
    motor_shield.setM1Speed(200);
}

void loop() {
    delay(1000);
    Serial.println(encoder_pulses);
    /*
    int pulses = encoder_pulses - encoder_pulses_prev;
    if (pulses != 0){
        encoder_pulses_prev = encoder_pulses;
        Serial.println(encoder_pulses);
    }
    motor_shield.getM1CurrentMilliamps();
    */
}
