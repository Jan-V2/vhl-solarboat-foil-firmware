#include "DualVNH5019MotorShield.h"
#include "types.h"

#define PETER_FILTER // peters filter voor potmeter
#define SLOWSTART
//#define FINE_TUNE_POT

#define ENC_PIN_1 PB2
#define ENC_PIN_2 PB1
#define ENC2_PIN_1 PB15
#define ENC2_PIN_2 PB14

//#define PRINT_PULSE
#define HOME_DEBUG
#define us_in_second 1000000

// PWM + means backwards, and - means forwards

// todo change pwm frequency
// todo add build flags like -Wno-unused-variables

DualVNH5019MotorShield motor_shield;

volatile int encoder1_pulses = 0;
volatile int encoder2_pulses = 0;

// vars for serial readout
String read_buffer = "";
String cmd = "";
const int bufferSize = 200;
const char cmd_sep = '|';

bool drawing_graph = false;

// vars for pot control
bool has_homed = false;
bool homed_recently = false;
int home_pos = -100;
float offset_max = 4 * 65; 
float pulses_max = 19000; 
float analog_max = 1024;

int offset_fixed = -6; // twists the foil left or right

// analog pins
int offset_pin = A4;
int pos_pin = A2;

int m2_current_offset = 408; // bodges bug in current readout

int delay_time = 1; // delay time in loop
ulong microsLast_0; // used for pwm
ulong microsLast_1; // used for pwm
int error_max = 3; // if error less than error_max pwm = 0


#ifdef PETER_FILTER
int setpoint_filtered = 0;
float filter_ratio = 0.99;
#else
int pos_min_change = 200;
#endif

#ifdef FINE_TUNE_POT
int off_min_change = 20;
#endif


static struct {
    float p_term = 2;
    float i_term = 0.00001;
    float d_term = 0.000005;

    int setpoint = 0;
    int offset = 0; // voor finetune pot
    float pref_error = 0;

    int pwm_min = 40;
    float out_min = -360, out_max = 360;

    float i_term_result = 0;
    float output = 0;
    int pwm = 0;
    int pwm_last = 0;
    
    int home_pwm_high = 400;
    int home_pwm_low = 70;
} motor_0;

static struct {
    float p_term = 2;
    float i_term = 0.00001;
    float d_term = 0.000005;

    int setpoint = 0;
    int offset = 0; // voor finetune pot
    float pref_error = 0;

    int pwm_min = 40;
    float out_min = -360, out_max = 360;
 
    float i_term_result = 0;
    float output = 0;
    int pwm = 0;
    int pwm_last = 0;
    
    int home_pwm_high = 400;
    int home_pwm_low = 70;
} motor_1;

static struct {
    int len = 5;
    float prev_vals[5] = {0,0,0,0,0};
    int idx = 0;
    int pref_pulses = 0;
} speed_calc_data;


void encoder_ISR() {
    if (digitalRead(ENC_PIN_2)) {
        encoder1_pulses++;
    } else {
        encoder1_pulses--;
    }
}

void encoder2_ISR() {
    if (digitalRead(ENC2_PIN_2)) {
        encoder2_pulses++;
    } else {
        encoder2_pulses--;
    }
}

void compute_pid_0() {
    float error = (float)(motor_0.setpoint + motor_0.offset - encoder1_pulses + offset_fixed);
    if (error > error_max * -1 && error < error_max) {
        motor_shield.setM1Speed(0);
        motor_0.pwm = 0;
        if (drawing_graph){
            drawing_graph = false;
            Serial.println("DONE GRAPH|");
        }
        return;
    }

    ulong cycle_time = micros() - microsLast_0;

    motor_0.i_term_result += motor_0.i_term * error * (float) cycle_time;
    if (motor_0.i_term_result > motor_0.out_max)
        motor_0.i_term_result = motor_0.out_max;       // that the I term from PID gets too big
    else if (motor_0.i_term_result < motor_0.out_min)
        motor_0.i_term_result = motor_0.out_min;

    float speed_error = (error - motor_0.pref_error) / (float)cycle_time;
    // PID motor_0.output is the sum of P I and D values
    motor_0.output = motor_0.p_term * error + motor_0.i_term_result - motor_0.d_term * speed_error;
    motor_0.pref_error = error;
    
    if (motor_0.output > 0) {
        if (motor_0.output > motor_0.out_max)
            motor_0.output = motor_0.out_max;
        motor_0.pwm = (int)motor_0.output + motor_0.pwm_min;
    } else {
        if (motor_0.output < motor_0.out_min)
            motor_0.output = motor_0.out_min;
        motor_0.pwm = (int)motor_0.output - motor_0.pwm_min;
    }

#ifdef SLOWSTART
    int pwm_diff = motor_0.pwm_last - motor_0.pwm;
    if (pwm_diff > 10 || pwm_diff < -10){
        motor_0.pwm_last = motor_0.pwm;
    }else{
        if (pwm_diff > 0){
            motor_0.pwm_last += 10;
        }else{
            motor_0.pwm_last -= 10;
        }
    }
    motor_shield.setM1Speed(motor_0.pwm_last);
#else
    motor_shield.setM1Speed(motor_0.pwm);
#endif
    microsLast_0 = micros();
}

void compute_pid_1() {
    float error = (float)(motor_1.setpoint + motor_1.offset - encoder2_pulses - offset_fixed);
    if (error > error_max * -1 && error < error_max) {
        motor_shield.setM2Speed(0);
        motor_1.pwm = 0;
        if (drawing_graph){
            drawing_graph = false;
            Serial.println("DONE GRAPH|");
        }
        return;
    }

    ulong cycle_time = micros() - microsLast_1;

    motor_1.i_term_result += motor_1.i_term * error * (float) cycle_time;
    if (motor_1.i_term_result > motor_1.out_max)
        motor_1.i_term_result = motor_1.out_max;       // that the I term from PID gets too big
    else if (motor_1.i_term_result < motor_1.out_min)
        motor_1.i_term_result = motor_1.out_min;

    float speed_error = (error - motor_1.pref_error) / (float)cycle_time;
    // PID motor_1.output is the sum of P I and D values
    motor_1.output = motor_1.p_term * error + motor_1.i_term_result - motor_1.d_term * speed_error;
    motor_1.pref_error = error;


    if (motor_1.output > 0) {
        if (motor_1.output > motor_1.out_max)
            motor_1.output = motor_1.out_max;
        motor_1.pwm = (int)motor_1.output + motor_1.pwm_min;
    } else {
        if (motor_1.output < motor_1.out_min)
            motor_1.output = motor_1.out_min;
        motor_1.pwm = (int)motor_1.output - motor_1.pwm_min;
    }
    
    
#ifdef SLOWSTART
    // changes pwm by at most 10 each delay_time ms
    int pwm_diff = motor_1.pwm_last - motor_1.pwm;
    if (pwm_diff > 10 || pwm_diff < -10){
        motor_1.pwm_last = motor_1.pwm;
    }else{
        if (pwm_diff > 0){
            motor_1.pwm_last += 10;
        }else{
            motor_1.pwm_last -= 10;
        }
    }
    motor_shield.setM1Speed(motor_1.pwm_last);
#else
    motor_shield.setM1Speed(motor_1.pwm);
#endif
    
    microsLast_1 = micros();
}

int sample_motor_current(int motor_num){
    // samples the motor_0 current 3 times 1 ms apart and take the average.
    // this is done in order to improve stablility
    int current = 0;
    int current1 = 0;
    int current2 = 0;
    
    if (motor_num){
        //todo fix this
        current += motor_shield.getM2CurrentMilliamps();
        delay(1);
        current2 += motor_shield.getM2CurrentMilliamps();
        delay(1);
        current2+= motor_shield.getM2CurrentMilliamps();
        delay(1);
        current = current + current1 + current2;
        current = current - m2_current_offset;
        // m2 has offset  for some reason
        
    }else{
        current += motor_shield.getM1CurrentMilliamps();
        delay(1);
        current += motor_shield.getM1CurrentMilliamps();
        delay(1);
        current += motor_shield.getM1CurrentMilliamps();
        delay(1);
    }
    
    return current / 3;
}

void home_actuators() {
    // homing sequence blocking routine
    // TODO add stall detection
#ifdef HOME_DEBUG
    Serial.println("homing");
#endif
    motor_shield.setM1Speed(motor_0.home_pwm_high);
    motor_shield.setM2Speed(motor_0.home_pwm_high);
    int motor_current_1 = 0;
    int motor_current_2 = 0;
    delay(100);
    motor_current_1 = sample_motor_current(0);
    motor_current_2 = sample_motor_current(1);
    while(motor_current_1 > 10 || motor_current_2 > 40){
#ifdef HOME_DEBUG
        //Serial.print(encoder1_pulses);
        //Serial.print(" ");
        //Serial.println(encoder2_pulses);
        Serial.print(motor_current_1);
        Serial.print(" ");
        Serial.println(motor_current_2);
#endif
       delay(100);
       motor_current_1 = sample_motor_current(0);
       motor_current_2 = sample_motor_current(1);
    }
#ifdef HOME_DEBUG
    Serial.println("stage 1 done");
#endif
    motor_shield.setM1Speed( -1 * motor_0.home_pwm_high);
    motor_shield.setM2Speed( -1 * motor_0.home_pwm_high);
    delay(1000);
    motor_shield.setM1Speed(motor_0.home_pwm_high);
    motor_shield.setM2Speed(motor_0.home_pwm_high);
    delay(20);
    motor_current_1 = sample_motor_current(0);
    motor_current_2 = sample_motor_current(1);
    while(motor_current_1 > 10 || motor_current_2 > 40){
#ifdef HOME_DEBUG
       Serial.println("t");
#endif
       delay(97);
       motor_current_1 = sample_motor_current(0);
       motor_current_2 = sample_motor_current(1);
    }
    encoder1_pulses = 0;
    encoder2_pulses = 0;
    motor_0.setpoint = 0;
    motor_1.setpoint = 0;
    motor_shield.setM1Speed(0);
    motor_shield.setM2Speed(0);
#ifdef HOME_DEBUG
    Serial.println("homed");
#endif    
}

void process_serial_cmd() {
    // todo error handeling for casts
    while (Serial.available() && read_buffer.length() < bufferSize) {
        // grabs a char and appends it to the buffer
        char c = Serial.read();
        read_buffer += c;
    }

    if ((read_buffer.length() >= bufferSize) || read_buffer.indexOf(cmd_sep) > 0) {
        int cmd_sep_idx = read_buffer.indexOf(cmd_sep);
        int setpoint;
        
        if (cmd_sep_idx > 0) {
            cmd = read_buffer.substring(0, cmd_sep_idx);
            read_buffer = read_buffer.substring(cmd_sep_idx + 1);
            if (cmd.indexOf("HOME") > -1){
                Serial.print("ACK HOME|");
                home_actuators();
                Serial.print("DONE HOME|");
            }else if (cmd.indexOf("MOVE") > -1){
                Serial.print("ACK MOVE|");
                // 5 is cmd len + space
                setpoint = cmd.substring(cmd.indexOf("MOVE") + 5).toInt();
                motor_0.setpoint = setpoint;
                motor_1.setpoint = setpoint;
#ifdef CMD_DEBUG
                Serial.println(setpoint);
#endif
            }else if (cmd.indexOf("GRAPH") > -1){
                Serial.println("ACK GRAPH|");
                // 5 is cmd len + space
                setpoint = cmd.substring(cmd.indexOf("GRAPH") + 6).toInt();
                motor_0.setpoint = setpoint;
                motor_1.setpoint = setpoint;
                drawing_graph = true;
            }else if (cmd.indexOf("STOP") > -1){
                setpoint = encoder1_pulses;
                motor_0.setpoint = setpoint;
                motor_1.setpoint = setpoint;
                drawing_graph = false;
                Serial.println("DONE STOP|");
            }
            
        } else if (cmd_sep_idx == 0) {
            read_buffer = read_buffer.substring(1);
        } else {
            //clears buffer if full and no cmd
            read_buffer = "";
        }
    }
}


void setup() {
    Serial.begin(115200);
    // configure interrupts and timers
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_1), encoder_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC2_PIN_1), encoder2_ISR, FALLING);
    motor_shield.init();
    delay(10);

    Serial.println("ready");
}

void loop() {
    delay(delay_time);
    process_serial_cmd();
    
#ifdef PRINT_PULSE
    Serial.print(encoder1_pulses);
    Serial.print(" ");
    Serial.println(encoder2_pulses);
#endif
    if (has_homed){
        compute_pid_0();
        compute_pid_1();
    }
    
    
#ifdef PRINT_A
    Serial.print(analogRead(pos_pin));
    Serial.print(" ");
    Serial.println(analogRead(offset_pin));
#endif

    int pos = (int)(((float)(analogRead(pos_pin)) / analog_max) * pulses_max) * -1;
    if (pos > home_pos){
        if (!homed_recently){
            home_actuators();
            has_homed = true;
            homed_recently = true;
        }
    }else{
        homed_recently = false;
#ifdef PETER_FILTER
        setpoint_filtered = pos * (1 - filter_ratio) + setpoint_filtered * filter_ratio;
        motor_0.setpoint = setpoint_filtered;
        motor_1.setpoint = setpoint_filtered;
#else
        if (motor_0.setpoint + pos_min_change < pos || motor_0.setpoint - pos_min_change > pos){
            motor_0.setpoint = pos;
            motor_1.setpoint = pos;
        }
#endif
    }

#ifdef FINE_TUNE_POT
    int offset_val = analogRead(offset_pin) * 2 - analog_max;
    int offset = (int)(((float)(offset_val) / analog_max) * offset_max) * -1;
    if (motor_0.offset + off_min_change < offset || motor_0.offset - off_min_change > offset){
        motor_0.offset = offset;
        motor_1.offset = offset;
    }
#endif
    

    if (drawing_graph){
        Serial.print(motor_0.pwm);
        Serial.print(' ');
        Serial.println(encoder1_pulses - motor_0.setpoint);
    }
}
