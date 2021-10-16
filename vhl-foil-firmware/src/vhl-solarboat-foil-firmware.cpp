
#include "motor_config.h"
#include "types.h"
#include "mcp2515.h"

#define ENC_L_P1 PB2
#define ENC_L_P2 PB1
#define ENC_R_P1 PB15
#define ENC_R_P2 PB14

//#define PRINT_PULSE
#define HOME_DEBUG
#define us_in_second 1000000

// todo change pwm frequency
// todo move cmd done acknowledge
// todo can toevoegen
// todo smoothing toevoegen

int delay_time = 100;

String read_buffer = "";
String cmd = "";
const int bufferSize = 200;
const char cmd_sep = '|';
int error_max = 2;

bool has_homed = false;
float pulses_max = 19000;

Motor motor_l = get_motor(M_LINKS);
Motor motor_r = get_motor(M_RECHTS);

void ENC_L_P1_ISR() {
    bool p1 = digitalRead(ENC_L_P1);
    bool p2 = digitalRead(ENC_L_P2);
    if (p1 == p2) {
        motor_l.encoder_pulses--;
    } else {
        motor_l.encoder_pulses++;
    }
}

void ENC_L_P2_ISR() {
    bool p1 = digitalRead(ENC_L_P1);
    bool p2 = digitalRead(ENC_L_P2);
    if (p1 == p2) {
        motor_l.encoder_pulses++;
    } else {
        motor_l.encoder_pulses--;
    }
}

void ENC_R_P1_ISR() {
    bool p1 = digitalRead(ENC_R_P1);
    bool p2 = digitalRead(ENC_R_P2);
    if (p1 == p2) {
        motor_l.encoder_pulses--;
    } else {
        motor_l.encoder_pulses++;
    }
}

void ENC_R_P2_ISR() {
    bool p1 = digitalRead(ENC_R_P1);
    bool p2 = digitalRead(ENC_R_P2);
    if (p1 == p2) {
        motor_l.encoder_pulses++;
    } else {
        motor_l.encoder_pulses--;
    }
}

void compute_pid(Motor m) {
    m.offset_fixed = -1 * offset_fixed_global;
    float error =
        (float)(m.setpoint + m.offset - m.encoder_pulses - offset_fixed_global);
    if (error > error_max * -1 && error < error_max) {
        m.set_speed(0);
        m.pwm = 0;
        return;
    }

    ulong cycle_time = micros() - m.last_cycle;

    m.i_term_result += m.i_term * error * (float)cycle_time;
    if (m.i_term_result > m.out_max)
        m.i_term_result = m.out_max;  // that the I term from PID gets too big
    else if (m.i_term_result < m.out_min)
        m.i_term_result = m.out_min;

    float speed = (error - m.pref_error) / (float)cycle_time;
    m.pid_out =
        m.p_term * error + m.i_term_result -
        m.d_term * speed;  // PID m.pid_out is the sum of P I and D values

    m.pref_error = error;

    if (m.pid_out > 0) {
        if (m.pid_out > m.out_max) m.pid_out = m.out_max;
        m.pwm = (int)m.pid_out + m.pwm_min;
    } else {
        if (m.pid_out < m.out_min) m.pid_out = m.out_min;
        m.pwm = (int)m.pid_out - m.pwm_min;
    }
    m.set_speed(m.pwm);
    m.last_cycle = micros();
}

int sample_motor_current(Motor m) {
    // samples the motor_0 current 3 times 1 ms apart and take the average.
    // this is done in order to improve stablility
    int current = 0;

    current += m.get_current();
    delay(1);
    current += m.get_current();
    delay(1);
    current += m.get_current();
    delay(1);
    current = current - m.current_offset;  // m1 has offset for some reason
    return current / 3;
}

void home_actuators() {
    // homing sequence blocking routine

    int motor_0_stop_current = 10;
    int motor_1_stop_current = 40;

#ifdef HOME_DEBUG
    Serial.println("homing");
#endif
    motor_l.set_speed(motor_l.home_pwm_high);
    motor_r.set_speed(motor_l.home_pwm_high);
    int motor_current_0 = 0;
    int motor_current_1 = 0;
    delay(100);
    motor_current_0 = sample_motor_current(motor_l);
    motor_current_1 = sample_motor_current(motor_r);
    while (motor_current_0 > motor_0_stop_current ||
           motor_current_1 > motor_1_stop_current) {
#ifdef HOME_DEBUG
        // Serial.print(encoder1_pulses);
        // Serial.print(" ");
        // Serial.println(encoder2_pulses);
        Serial.print(motor_current_0);
        Serial.print(" ");
        Serial.println(motor_current_1);
#endif
        delay(100);
        motor_current_0 = sample_motor_current(motor_l);
        motor_current_1 = sample_motor_current(motor_r);
    }
#ifdef HOME_DEBUG
    Serial.println("stage 1 done");
#endif
    motor_l.set_speed(-1 * motor_l.home_pwm_high);
    motor_r.set_speed(-1 * motor_l.home_pwm_high);
    delay(1000);
    motor_l.set_speed(motor_l.home_pwm_high);
    motor_r.set_speed(motor_l.home_pwm_high);
    delay(20);
    motor_current_0 = sample_motor_current(motor_l);
    motor_current_1 = sample_motor_current(motor_r);

    while (motor_current_0 > motor_0_stop_current ||
           motor_current_1 > motor_1_stop_current) {
#ifdef HOME_DEBUG
        Serial.println("t");
#endif
        delay(97);
        motor_current_0 = sample_motor_current(motor_l);
        motor_current_1 = sample_motor_current(motor_r);
    }
    motor_l.encoder_pulses = 0;
    motor_r.encoder_pulses = 0;
    motor_l.setpoint = 0;
    motor_r.setpoint = 0;
    motor_l.set_speed(0);
    motor_r.set_speed(0);
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

    if ((read_buffer.length() >= bufferSize) ||
        read_buffer.indexOf(cmd_sep) > 0) {
        int cmd_sep_idx = read_buffer.indexOf(cmd_sep);
        int setpoint;

        if (cmd_sep_idx > 0) {
            cmd = read_buffer.substring(0, cmd_sep_idx);
            read_buffer = read_buffer.substring(cmd_sep_idx + 1);
            if (cmd.indexOf("HOME") > -1) {
                Serial.print("ACK HOME|");
                home_actuators();
                Serial.print("DONE HOME|");
            } else if (cmd.indexOf("MOVE") > -1) {
                Serial.print("ACK MOVE|");
                // 5 is cmd len + space
                setpoint = cmd.substring(cmd.indexOf("MOVE") + 5).toInt();
                motor_l.setpoint = setpoint;
                motor_r.setpoint = setpoint;
#ifdef CMD_DEBUG
                Serial.println(setpoint);
#endif
            } else if (cmd.indexOf("STOP") > -1) {
                setpoint = motor_l.encoder_pulses;
                motor_l.setpoint = setpoint;
                motor_r.setpoint = setpoint;
                Serial.println("DONE STOP|");
            }
        } else if (cmd_sep_idx == 0) {
            read_buffer = read_buffer.substring(1);
        } else {
            // clears buffer if full and no cmd
            read_buffer = "";
        }
    }
}

void setup() {
    Serial.begin(115200);
    // configure interrupts and timers

    // TODO is pullup wel nodig?
    pinMode(ENC_L_P1, INPUT_PULLUP);
    pinMode(ENC_L_P2, INPUT_PULLUP);
    pinMode(ENC_R_P1, INPUT_PULLUP);
    pinMode(ENC_R_P2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_L_P1), ENC_L_P1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_P2), ENC_L_P2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_P1), ENC_R_P1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_P2), ENC_R_P2_ISR, CHANGE);

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
    if (has_homed) {
        compute_pid(motor_l);
        compute_pid(motor_r);
    }

#ifdef PRINT_A
    Serial.print(analogRead(pos_pin));
    Serial.print(" ");
    Serial.println(analogRead(offset_pin));
#endif
}
