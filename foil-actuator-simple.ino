#include "DualVNH5019MotorShield.h"
#include "types.h"

#define ENC_PIN_1 11
#define ENC_PIN_2 13

#define us_in_second 1000000

// PWM + means backwards, and - means forwards

// todo change pwm frequency
// todo add build flags like -Wno-unused-variables

DualVNH5019MotorShield motor_shield;

volatile int encoder_pulses = 0;
static volatile int encoder_pulses_prev = encoder_pulses;

// todo refactor this so that the motor control code is in it's own file (class maybe?)
// todo move cmd done acknowledge

int delay_time = 100;
ulong microsLast;

String read_buffer = "";
String cmd = "";
const int bufferSize = 200;
const char cmd_sep = '|';

bool drawing_graph = false;

static struct {
    float p_term = 2;
    float i_term = 0.0000;
    float d_term = 0.000005;

    int setpoint = encoder_pulses;// todo make this encoder data?

    int pwm_min = 40;
    float out_min = -360, out_max = 360;

    float i_term_result = 0;
    float output = 0;
    float pwm = 0;
    
    int home_pwm_high = 400;
    int home_pwm_low = 70;
} motor;

static struct {
    int len = 5;
    float prev_vals[5] = {0,0,0,0,0};
    int idx = 0;
    int pref_pulses = 0;
} speed_calc_data;


void encoder_ISR() {
    if (digitalRead(ENC_PIN_2)) {
        encoder_pulses++;
    } else {
        encoder_pulses--;
    }
}

void compute_pid() {
    float error = (float)(motor.setpoint - encoder_pulses);
    if (error > -2 && error < 2) {
        motor_shield.setM1Speed(0);
        motor.pwm = 0;
        if (drawing_graph){
            drawing_graph = false;
            Serial.println("DONE GRAPH|");
        }
        return;
    }

    ulong cycle_time = micros() - microsLast;

    motor.i_term_result += motor.i_term * error * (float) cycle_time;
    if (motor.i_term_result > motor.out_max)
        motor.i_term_result = motor.out_max;       // that the I term from PID gets too big
    else if (motor.i_term_result < motor.out_min)
        motor.i_term_result = motor.out_min;

    int speed = (encoder_pulses - encoder_pulses_prev) / cycle_time;
    motor.output = motor.p_term * error + motor.i_term_result - motor.d_term * (float)speed;// PID motor.output is the sum of P I and D values

    encoder_pulses_prev = encoder_pulses;

    if (motor.output > 0) {
        if (motor.output > motor.out_max)
            motor.output = motor.out_max;
        motor.pwm = (int)motor.output + motor.pwm_min;
    } else {
        if (motor.output < motor.out_min)
            motor.output = motor.out_min;
        motor.pwm = (int)motor.output - motor.pwm_min;
    }
    motor_shield.setM1Speed(motor.pwm);
    microsLast = micros();
}

int sample_motor_current(){
    // samples the motor current 3 times 1 ms apart and take the average.
    // this is done in order to improve stablility
    int current = 0;
    current += motor_shield.getM1CurrentMilliamps();
    delay(1);
    current += motor_shield.getM1CurrentMilliamps();
    delay(1);
    current += motor_shield.getM1CurrentMilliamps();
    delay(1);
    return current / 3;
}

void home_actuator() {
    // homing sequence blocking routine
    // TODO add stall detection
#ifdef HOME_DEBUG
    Serial.println("homing");
#endif
    motor_shield.setM1Speed(motor.home_pwm_high);
    int motor_current = 0;
    //encoder_pulses_prev = encoder_pulses;
    delay(100);
    motor_current = sample_motor_current();
    while(motor_current > 10){
       //encoder_pulses_prev = encoder_pulses;
#ifdef HOME_DEBUG
       Serial.println(motor_current);
#endif
       delay(97);
       motor_current = sample_motor_current();
    }
#ifdef HOME_DEBUG
    Serial.println("stage 1 done");
#endif
    motor_shield.setM1Speed( -1 * motor.home_pwm_low);
    delay(1000);
    motor_shield.setM1Speed(motor.home_pwm_low);
    delay(20);
    motor_current = sample_motor_current();
    while(motor_current > 10){
       //encoder_pulses_prev = encoder_pulses;
#ifdef HOME_DEBUG
       Serial.println(motor_current);
#endif
       delay(97);
       motor_current = sample_motor_current();
    }
    encoder_pulses = 0;
    motor.setpoint = 0;
    motor_shield.setM1Speed(0);
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

        if (cmd_sep_idx > 0) {
            cmd = read_buffer.substring(0, cmd_sep_idx);
            read_buffer = read_buffer.substring(cmd_sep_idx + 1);
            if (cmd.indexOf("HOME") > -1){
                Serial.print("ACK HOME|");
                home_actuator();
                Serial.print("DONE HOME|");
            }else if (cmd.indexOf("MOVE") > -1){
                Serial.print("ACK MOVE|");
                // 5 is cmd len + space
                motor.setpoint = cmd.substring(cmd.indexOf("MOVE") + 5).toInt();
#ifdef CMD_DEBUG
                Serial.println(motor.setpoint);
#endif
            }else if (cmd.indexOf("GRAPH") > -1){
                Serial.println("ACK GRAPH|");
                // 5 is cmd len + space
                motor.setpoint = cmd.substring(cmd.indexOf("GRAPH") + 6).toInt();
                drawing_graph = true;
            }else if (cmd.indexOf("STOP") > -1){
                motor.setpoint = encoder_pulses;
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
    motor_shield.init();
    Serial.println("ready");
}

void loop() {
    delay(delay_time);
    process_serial_cmd();

    compute_pid();
    if (drawing_graph){
        Serial.print(motor.pwm);
        Serial.print(' ');
        Serial.println(encoder_pulses - motor.setpoint);
    }
}
