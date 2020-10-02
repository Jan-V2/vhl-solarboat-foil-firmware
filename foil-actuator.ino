#include "DualVNH5019MotorShield.h"
#include "types.h"

#define ENC_PIN_1 11
#define ENC_PIN_2 13

#define us_in_second 1000000
#define systick_per_sec 1000

// PWM + means backwards, and - means forwards

// todo change pwm frequency

DualVNH5019MotorShield motor_shield;

int encoder_pulses = 0;

static volatile int encoder_pulses_prev = encoder_pulses;
static volatile u32 systick_count = 0;
static volatile float speed_pulses_per_sec = 0.0;


static struct {
    int len = 5;
    float prev_vals[5] = {0,0,0,0,0};
    int idx = 0;
    int systicks_per_calc = systick_per_sec / 5; // 10 hertz
    int pref_pulses = 0;
}speed_calc_data;


void encoder_ISR(){
    if (digitalRead(ENC_PIN_2)){
        encoder_pulses--;
    }else{
        encoder_pulses++;
    }
}


void HAL_SYSTICK_Callback(void) {
    if (systick_count++ == speed_calc_data.systicks_per_calc) {
        //Serial.println(millis());
        systick_count = 0;
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
}


void setup() {
    Serial.begin(115200);
    
    // configure interrupts and timers 
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_1), encoder_ISR, FALLING);
    

    
    motor_shield.init();
    motor_shield.setM1Speed(200);
}

void loop() {
    delay(500);
    Serial.println(speed_pulses_per_sec);
    /*
    int pulses = encoder_pulses - encoder_pulses_prev;
    if (pulses != 0){
        encoder_pulses_prev = encoder_pulses;
        Serial.println(encoder_pulses);
    }
    motor_shield.getM1CurrentMilliamps();
    */
}
