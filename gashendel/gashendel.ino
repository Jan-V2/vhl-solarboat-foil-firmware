#include "types.h"
#define IDLE_TIMEOUT

#define DEBUG

// 1000 us backwards max
// 1500 us neutral
// 2000 us forward max
const int analog_in = A5;
const int btn_in = A2;
const int pwm_pin = D2;
const int permission_pin = PF_1;
const int temp_permission_PIN = PA_8;
const int permission_time = 500;

bool motorConected;
ulong lastPermission = 0;

#ifdef IDLE_TIMEOUT
const int idle_time_ms = 30000;
ulong last_idle_ms = millis();
bool is_idle = false;
#endif

const int adc_low = 420;
const int adc_high = 900;
const int range = adc_high - adc_low;
const int deadzone_low = (int)((float)range * 0.05);
const int deadzone_high = (int)((float)range * 0.95);
const int samples = 10;

HardwareTimer *pwm_timer;
const TimerCompareFormat_t pwm_format = RESOLUTION_12B_COMPARE_FORMAT;
const int pwm_max = 4095;

// these variables change the pwm timings
const int pwm_backwards = (int)((float)pwm_max / 20.0);
const int pwm_forward = (int)((float)pwm_max / 10.0);
const int pwm_neutral = (int)((float)pwm_max / 13.3333);

uint32_t pwm_channel = 0;
const int blinkspeed = 500;

enum State {forwards, backwards, neutral_btn, neutral, bootup};
State current_state = bootup;

int throttle = 0;


bool in_neutral() {
    return throttle == 0;
}

void set_pwm(int compare, bool forwards) {
    // sets pwm of output pin accepts between 0 and range
    int pwm_out;
    if(forwards) {
        pwm_out = map(compare, 0, range, pwm_neutral, pwm_forward);
    } else {
        pwm_out = map(compare, 0, range, pwm_neutral, pwm_backwards);
    }
    pwm_timer->setCaptureCompare(pwm_channel, pwm_out, pwm_format);
}

void set_pwm_neutral() {
    set_pwm(0, true);
}

int read_throttle() {
    int acc = 0;
    for(int i=0; i < samples; i++ ) {
        acc += analogRead(analog_in);
        delay(1);
    }
    acc = acc / samples;
    acc = map(acc, adc_low, adc_high, 0, range);

    if(acc <= deadzone_low) {
        acc = 0;
    } else if(acc >= deadzone_high) {
        acc = range;
    }
    return acc;
}

void setup() {
    // Automatically retrieve TIM instance and pwm_channel associated to pin
    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwm_pin), PinMap_PWM);
    pwm_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwm_pin), PinMap_PWM));

    pwm_timer = new HardwareTimer(Instance);

    pwm_timer->setMode(pwm_channel, TIMER_OUTPUT_COMPARE_PWM1, pwm_pin);
    pwm_timer->setOverflow(50, HERTZ_FORMAT);
    pwm_timer->resume();

    set_pwm_neutral();

    pinMode(btn_in, INPUT);
    pinMode(analog_in, INPUT);
    pinMode(permission_pin, OUTPUT);
    pinMode(temp_permission_PIN, INPUT_PULLDOWN);
#ifdef DEBUG
    Serial.begin(115200);
#endif
    digitalWrite(LED_BUILTIN, HIGH);

}

void loop() {
#ifdef IDLE_TIMEOUT
    if (is_idle) {
              if (digitalRead(btn_in)) {
            is_idle = false;
            last_idle_ms = millis();
        }
    } else {
        if(millis() - last_idle_ms > idle_time_ms && (current_state == neutral || current_state == bootup) ) {
            is_idle = true;
        }
    }
    if(!is_idle) {
#endif
        throttle = read_throttle();
        if (current_state == bootup) {
            if(in_neutral()) {
                // to make sure the boat doesn't move until the throttle is set to neutral
                current_state = neutral;
            }
        } else if (current_state == forwards) {
            if(in_neutral()) {
                if (digitalRead(btn_in)) {
                    current_state = neutral_btn;
                } else {
                    current_state = neutral;
                }
            } else {
                set_pwm(throttle, true);
            }
        } else if (current_state == neutral) {
         motorConected = digitalRead(temp_permission_PIN);
         if((digitalRead(btn_in) || !in_neutral())){
                  lastPermission = millis();
                  digitalWrite(permission_pin, LOW) ;
                }
            if(in_neutral()) {
                set_pwm_neutral();
                if(motorConected){
                digitalWrite(permission_pin, HIGH) ;
                }
                
                if (digitalRead(btn_in) && motorConected == LOW || millis() - lastPermission > permission_time) {
                    current_state = neutral_btn;
                     
                }
            } else if(motorConected == LOW || millis() - lastPermission > permission_time){
                current_state = forwards;
                set_pwm(throttle, true);
                
            }
        } else if (current_state == neutral_btn) {
            if(in_neutral()) {
                set_pwm_neutral();
                if (!digitalRead(btn_in)) {
                    current_state = neutral;
                }
            } else {
                current_state = backwards;
                set_pwm(throttle, false);
            }
        } else if (current_state == backwards) {
            if(in_neutral()) {
                if (digitalRead(btn_in)) {
                    current_state = neutral_btn;
                } else {
                    current_state = neutral;
                }
            } else {
                set_pwm(throttle, false);
            }
        }
#ifdef IDLE_TIMEOUT
        if(!(current_state == neutral || current_state == bootup)) {
            last_idle_ms = millis();
        }
    }
#endif

#ifdef DEBUG
    Serial.print(throttle);
    Serial.print(" ");
    Serial.print(digitalRead(btn_in));
    Serial.print(" ");
    Serial.println(in_neutral());
    Serial.println(current_state);

#endif

    delay(1);
}
