#include "DualVNH5019MotorShield.h"
#include "types.h"

#define ENC_PIN_1 11
#define ENC_PIN_2 13

#define us_in_second 1000000

// PWM + means backwards, and - means forwards

// todo change pwm frequency

DualVNH5019MotorShield motor_shield;

volatile int encoder_pulses = 0;
int last_encoder_pulsed = encoder_pulses;

static volatile int encoder_pulses_prev = encoder_pulses;
static volatile u32 systick_count = 0;
static volatile float speed_pulses_per_sec = 0.0;

int delay_time = 1;
float p_term = 1;
float i_term = 0.0000;
float d_term = 0.000005;

int setpoint = -1000;

int pwm_min = 40;
float out_min = -370, out_max = 370;

float i_term_result = 0;
float output = 0;
float pwm = 0;

ulong microsLast;

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
        encoder_pulses++;
    }else{
        encoder_pulses--;
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

void compute_pid()
{
    float error = (float)(setpoint - encoder_pulses);
    if (error > -2 && error < 2){
        motor_shield.setM1Speed(0);
        pwm = 0;
        return;
    }
    
    ulong cycle_time = micros() - microsLast;
    
    i_term_result += i_term * error * (float) cycle_time;
    if (i_term_result > out_max) 
        i_term_result = out_max;       // that the I term from PID gets too big
    else if (i_term_result < out_min) 
        i_term_result = out_min;
        
    int speed = (encoder_pulses - encoder_pulses_prev) / cycle_time; 
    output = p_term * error + i_term_result - d_term * (float)speed;// PID output is the sum of P I and D values
    
    encoder_pulses_prev = encoder_pulses;

    if (output > 0){
        if (output > out_max) 
            output = out_max;  
        pwm = (int)output + pwm_min;
    }else{
        if (output < out_min)
            output = out_min;          
        pwm = (int)output - pwm_min;
    }
    motor_shield.setM1Speed(pwm);
    
    microsLast = micros();
    
}


void setup() {
    Serial.begin(115200);
    
    // configure interrupts and timers 
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_1), encoder_ISR, FALLING);
    

    motor_shield.init();
}

int print_max = 10;
int print = 0;

void loop() {
    delay(delay_time);
    compute_pid();
    
    if (print == print_max){
        Serial.print(pwm);
        Serial.print(' ');
        Serial.println(encoder_pulses - setpoint);
        print = 0;
    }else{
        print++;
    }
}
