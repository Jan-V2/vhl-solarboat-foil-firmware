
#include "types.h"
#include "motor_config.h"

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



float error0_prev = 0;


// todo refactor this so that the motor_0 control code is in it's own file (class maybe?)
// todo move cmd done acknowledge

int delay_time = 100;
ulong microsLast_0;
ulong microsLast_1;

String read_buffer = "";
String cmd = "";
const int bufferSize = 200;
const char cmd_sep = '|';
int error_max = 2;

bool drawing_graph = false;

bool has_homed = false;
float pulses_max = 19000; 

Motor motor_0 = get_motor(M0);
Motor motor_1 = get_motor(M1);


static struct {
    int len = 5;
    float prev_vals[5] = {0,0,0,0,0};
    int idx = 0;
    int pref_pulses = 0;
} speed_calc_data;


void encoder0_ISR() {
    if (digitalRead(ENC_PIN_2)) {
        motor_0.encoder_pulses++;
    } else {
        motor_0.encoder_pulses--;
    }
}

void encoder1_ISR() {
    if (digitalRead(ENC2_PIN_2)) {
        motor_1.encoder_pulses++;
    } else {
        motor_1.encoder_pulses--;
    }
}





void compute_pid(Motor m) {
    m.offset_fixed = -1 * offset_fixed_global;
    float error = (float)(m.setpoint + m.offset - m.encoder_pulses - offset_fixed_global);
    if (error > error_max * -1 && error < error_max) {
        m.set_speed(0);
        m.pwm = 0;
        if (drawing_graph){
            drawing_graph = false;
            Serial.println("DONE GRAPH|");
        }
        return;
    }

    ulong cycle_time = micros() - microsLast_0;

    m.i_term_result += m.i_term * error * (float) cycle_time;
    if (m.i_term_result > m.out_max)
        m.i_term_result = m.out_max;       // that the I term from PID gets too big
    else if (m.i_term_result < m.out_min)
        m.i_term_result = m.out_min;

    float speed = (error - error0_prev) / (float)cycle_time;
    m.pid_out = m.p_term * error + m.i_term_result - m.d_term * speed;// PID m.pid_out is the sum of P I and D values

    error0_prev = error;

    if (m.pid_out > 0) {
        if (m.pid_out > m.out_max)
            m.pid_out = m.out_max;
        m.pwm = (int)m.pid_out + m.pwm_min;
    } else {
        if (m.pid_out < m.out_min)
            m.pid_out = m.out_min;
        m.pwm = (int)m.pid_out - m.pwm_min;
    }
    m.set_speed(m.pwm);
    microsLast_0 = micros();
}



int sample_motor_current(Motor m){
    // samples the motor_0 current 3 times 1 ms apart and take the average.
    // this is done in order to improve stablility
    int current = 0;
    
    current += m.get_current();
    delay(1);
    current += m.get_current();
    delay(1);
    current += m.get_current();
    delay(1);
    current = current - m.current_offset;// m1 has offset for some reason
    return current / 3;
}

void home_actuators() {
    // homing sequence blocking routine

    int motor_0_stop_current = 10;
    int motor_1_stop_current = 40;
    
#ifdef HOME_DEBUG
    Serial.println("homing");
#endif
    motor_0.set_speed(motor_0.home_pwm_high);
    motor_1.set_speed(motor_0.home_pwm_high);
    int motor_current_0 = 0;
    int motor_current_1 = 0;
    delay(100);
    motor_current_0 = sample_motor_current(motor_0);
    motor_current_1 = sample_motor_current(motor_1);
    while(motor_current_0 > motor_0_stop_current || 
        motor_current_1 > motor_1_stop_current){
#ifdef HOME_DEBUG
        //Serial.print(encoder1_pulses);
        //Serial.print(" ");
        //Serial.println(encoder2_pulses);
        Serial.print(motor_current_0);
        Serial.print(" ");
        Serial.println(motor_current_1);
#endif
       delay(100);
       motor_current_0 = sample_motor_current(motor_0);
       motor_current_1 = sample_motor_current(motor_1);
    }
#ifdef HOME_DEBUG
    Serial.println("stage 1 done");
#endif
    motor_0.set_speed( -1 * motor_0.home_pwm_high);
    motor_1.set_speed( -1 * motor_0.home_pwm_high);
    delay(1000);
    motor_0.set_speed(motor_0.home_pwm_high);
    motor_1.set_speed(motor_0.home_pwm_high);
    delay(20);
    motor_current_0 = sample_motor_current(motor_0);
    motor_current_1 = sample_motor_current(motor_1);
    
    while(motor_current_0 > motor_0_stop_current || 
        motor_current_1 > motor_1_stop_current){
#ifdef HOME_DEBUG
       Serial.println("t");
#endif
       delay(97);
       motor_current_0 = sample_motor_current(motor_0);
       motor_current_1 = sample_motor_current(motor_1);
    }
    motor_0.encoder_pulses = 0;
    motor_1.encoder_pulses = 0;
    motor_0.setpoint = 0;
    motor_1.setpoint = 0;
    motor_0.set_speed(0);
    motor_1.set_speed(0);
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
                setpoint = motor_0.encoder_pulses;
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
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_1), encoder0_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC2_PIN_1), encoder1_ISR, FALLING);
    motor_shield.init();
    delay(10);

    Serial.println("ready");
}

void loop() {
    delay(delay_time);
    process_serial_cmd();
    
#ifdef PRINT_PULSE
    Serial.print(motor_0.encoder_pulses);
    Serial.print(" ");
    Serial.println(motor_1.encoder_pulses);
#endif
    if (has_homed){
        compute_pid(motor_0);
        compute_pid(motor_1);
    }
    
    
#ifdef PRINT_A
    Serial.print(analogRead(pos_pin));
    Serial.print(" ");
    Serial.println(analogRead(offset_pin));
#endif
    
    
    
    if (drawing_graph){
        Serial.print(motor_0.pwm);
        Serial.print(' ');
        Serial.println(motor_0.encoder_pulses - motor_0.setpoint);
    }
}
