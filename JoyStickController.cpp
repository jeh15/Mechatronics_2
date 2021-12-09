#include "Arduino.h"
#include <Encoder.h>
#include "JoyStickController.h"
#include "math.h"

JoyStickController::JoyStickController(int pin_PWM_1, int pin_MOTOR_OUT1_1, int pin_MOTOR_OUT2_1, int pin_ENCODER_A_1,
                                       int pin_ENCODER_B_1, int pin_INDEX_1, float motor_RESOLUTION_1,
                                       int pin_HALL_ENCODER_1, float hall_RESOLUTION_1, int pin_JOY_STICK_1, int pin_JOY_STICK_BUTTON_1,
                                       float Kp_1, float Ki_1, float Kd_1, float K_joy_stick_1,
                                       int pin_PWM_2, int pin_MOTOR_OUT1_2, int pin_MOTOR_OUT2_2, int pin_ENCODER_A_2,
                                       int pin_ENCODER_B_2, int pin_INDEX_2, float motor_RESOLUTION_2,
                                       int pin_HALL_ENCODER_2, float hall_RESOLUTION_2, int pin_JOY_STICK_2, int pin_JOY_STICK_BUTTON_2,
                                       float Kp_2, float Ki_2, float Kd_2, float K_joy_stick_2)
        : encoder_1(pin_ENCODER_A_1, pin_ENCODER_B_1)
        , encoder_2(pin_ENCODER_A_2, pin_ENCODER_B_2){

    // Initialize Motor Pins:
    pinMode(pin_PWM_1, OUTPUT);
    pinMode(pin_MOTOR_OUT1_1, OUTPUT);
    pinMode(pin_MOTOR_OUT2_1, OUTPUT);
    pinMode(pin_PWM_2, OUTPUT);
    pinMode(pin_MOTOR_OUT1_2, OUTPUT);
    pinMode(pin_MOTOR_OUT2_2, OUTPUT);

    // Initialize Encoder Reset Pin:
    pinMode(pin_INDEX_1, INPUT);
    pinMode(pin_INDEX_2, INPUT);

    // Initialize Joy Stick Button Pin:
    pinMode(pin_JOY_STICK_BUTTON_1, INPUT);
    pinMode(pin_JOY_STICK_BUTTON_2, INPUT);

    // Motor Pins / Constants: MOTOR 1
    _pin_PWM_1 = pin_PWM_1;
    _pin_MOTOR_OUT1_1 = pin_MOTOR_OUT1_1;
    _pin_MOTOR_OUT2_1 = pin_MOTOR_OUT2_1;

    // Motor Pins / Constants: MOTOR 2
    _pin_PWM_2 = pin_PWM_2;
    _pin_MOTOR_OUT1_2 = pin_MOTOR_OUT1_2;
    _pin_MOTOR_OUT2_2 = pin_MOTOR_OUT2_2;

    // Encoder Pins / Constants: MOTOR 1
    _pin_ENCODER_A_1 = pin_ENCODER_A_1;
    _pin_ENCODER_B_1 = pin_ENCODER_B_1;
    _pin_INDEX_1 = pin_INDEX_1;
    _pin_HALL_ENCODER_1 = pin_HALL_ENCODER_1;

    // Encoder Pins / Constants: MOTOR 2
    _pin_ENCODER_A_2 = pin_ENCODER_A_2;
    _pin_ENCODER_B_2 = pin_ENCODER_B_2;
    _pin_INDEX_2 = pin_INDEX_2;
    _pin_HALL_ENCODER_2 = pin_HALL_ENCODER_2;

    // Joy Stick Pins:
    _pin_JOY_STICK_1 = pin_JOY_STICK_1;
    _pin_JOY_STICK_2 = pin_JOY_STICK_2;
    _pin_JOY_STICK_BUTTON_1 = pin_JOY_STICK_BUTTON_1;
    _pin_JOY_STICK_BUTTON_2 = pin_JOY_STICK_BUTTON_2;

    // Resolution of Encoder: MOTOR 1
    _motor_RESOLUTION_1 = motor_RESOLUTION_1;
    _hall_RESOLUTION_1 = hall_RESOLUTION_1;

    // Resolution of Encoder: MOTOR 2
    _motor_RESOLUTION_2 = motor_RESOLUTION_2;
    _hall_RESOLUTION_2 = hall_RESOLUTION_2;

    // Control Law Variables:
    _Kp_1 = Kp_1;
    _Ki_1 = Ki_1;
    _Kd_1 = Kd_1;
    _Kp_2 = Kp_2;
    _Ki_2 = Ki_2;
    _Kd_2 = Kd_2;
    _K_joy_stick_1 = K_joy_stick_1;
    _K_joy_stick_2 = K_joy_stick_2;
    _delay_1 = 0;
    _delay_2 = 0;
    _int_error_1 = 0;
    _error_1 = 0;
    _int_error_2 = 0;
    _error_2 = 0;
    _theta_hold_1 = 0;
    _theta_hold_2 = 0;

    // Joy Stick:
    _joy_stick_1 = 0;
    _joy_stick_2 = 0;
    _joy_stick_1_mode = 0;
    _joy_stick_2_mode = 0;

    // Bound Constants:
    _ERROR_BOUND = 0.05;
    _NOMINAL_JOY_STICK = 511.0;
    _DEAD_ZONE = 50.0;
    _SATURATION_LIMIT = 255.0;

    // Gain Schedule Variables:
    _GAIN_SCHEDULE_ENABLE = 0;
    _JOY_STICK_GAIN_1 = K_joy_stick_1;
    _JOY_STICK_GAIN_2 = K_joy_stick_2;
    _old_Button_State_1 = 0;
    _new_Button_State_1 = 0;

    // Mode Switch:
    _MODE_SWITCH = 0;
    _old_Button_State_2 = 0;
    _new_Button_State_2 = 0;

}

void JoyStickController::controllerMode(){
    // Control Law:
    _joy_stick_1 = _NOMINAL_JOY_STICK - analogRead(_pin_JOY_STICK_1);
    _joy_stick_2 = _NOMINAL_JOY_STICK - analogRead(_pin_JOY_STICK_2);

    // Gain Schedule:
    if(_GAIN_SCHEDULE_ENABLE == 1){
        _JOY_STICK_GAIN_1 = _K_joy_stick_1 * exp(-1 / 4 * abs(_nominal_Deflection_1 - analogRead(_pin_HALL_ENCODER_1)));
        _JOY_STICK_GAIN_2 = _K_joy_stick_2 * exp(-1 / 4 * abs(_nominal_Deflection_2 - analogRead(_pin_HALL_ENCODER_2)));
    }
    else{
        _JOY_STICK_GAIN_1 = _K_joy_stick_1;
        _JOY_STICK_GAIN_2 = _K_joy_stick_2;
    }

    // Hold Position: MOTOR 1
    if(_joy_stick_1 < _DEAD_ZONE && _joy_stick_1 > -1 * _DEAD_ZONE){
        _prev_error_1 = _error_1;
        _error_1 = _theta_hold_1 - _motor_RESOLUTION_1 * encoder_1.read();
        _diff_error_1 = _error_1 - _prev_error_1;
        _int_error_1 = _int_error_1 + _error_1;
        _controlInput_1 = _Kp_1 * _error_1 + (_Kd_1 * _diff_error_1) * (_delay_1 != 0) + _Ki_1 * _int_error_1;
        _delay_1 = 1;
        _joy_stick_1_mode = 0;
    }
    else{
        _theta_hold_1 = _motor_RESOLUTION_1 * encoder_1.read();
        _controlInput_1 = _JOY_STICK_GAIN_1 * _joy_stick_1;
        _error_1 = 0;
        _int_error_1 = 0;
        _delay_1 = 0;
        _joy_stick_1_mode = 1;
    }

    // Hold Position: MOTOR 2
    if(_joy_stick_2 < _DEAD_ZONE && _joy_stick_2 > -1 * _DEAD_ZONE){
        _prev_error_2 = _error_2;
        _error_2 = _theta_hold_2 - _motor_RESOLUTION_2 * encoder_2.read();
        _diff_error_2 = _error_2 - _prev_error_2;
        _int_error_2 = _int_error_2 + _error_2;
        _controlInput_2 = _Kp_2 * _error_2 + (_Kd_2 * _diff_error_2) * (_delay_2 != 0) + _Ki_2 * _int_error_2;
        _delay_2 = 1;
        _joy_stick_2_mode = 0;
    }
    else{
        _theta_hold_2 = _motor_RESOLUTION_2 * encoder_2.read();
        _controlInput_2 = _JOY_STICK_GAIN_2 * _joy_stick_2;
        _error_2 = 0;
        _int_error_2 = 0;
        _delay_2 = 0;
        _joy_stick_2_mode = 1;
    }

    // Saturate Control Input: MOTOR 1
    if (_controlInput_1 > _SATURATION_LIMIT) {
        _controlInput_1 = _SATURATION_LIMIT;
    }
    if (_controlInput_1 < -1 * _SATURATION_LIMIT) {
        _controlInput_1 = -1 * _SATURATION_LIMIT;
    }

    // Saturate Control Input: MOTOR 2
    if (_controlInput_2 > _SATURATION_LIMIT) {
        _controlInput_2 = _SATURATION_LIMIT;
    }
    if (_controlInput_2 < -1 * _SATURATION_LIMIT) {
        _controlInput_2 = -1 * _SATURATION_LIMIT;
    }

    // Motor 1 Control:
    if(_joy_stick_1_mode == 1) {
        if (_controlInput_1 > 0) {
            // CW Rotation:
            analogWrite(_pin_PWM_1, abs(_controlInput_1));
            digitalWrite(_pin_MOTOR_OUT1_1, HIGH);
            digitalWrite(_pin_MOTOR_OUT2_1, LOW);
        } else if (_controlInput_1 < 0) {
            // CCW Rotation:
            analogWrite(_pin_PWM_1, abs(_controlInput_1));
            digitalWrite(_pin_MOTOR_OUT1_1, LOW);
            digitalWrite(_pin_MOTOR_OUT2_1, HIGH);
        }
    }
    else{
        if (_error_1 < _ERROR_BOUND && _error_1 > -1 * _ERROR_BOUND) {
            // RESET INTEGRAL TERM:
            _int_error_1 = 0;
            // BREAK:
            analogWrite(_pin_PWM_1, 0);
            digitalWrite(_pin_MOTOR_OUT1_1, LOW);
            digitalWrite(_pin_MOTOR_OUT2_1, LOW);
        } else if (_controlInput_1 > 0) {
            // CW Rotation:
            analogWrite(_pin_PWM_1, abs(_controlInput_1));
            digitalWrite(_pin_MOTOR_OUT1_1, HIGH);
            digitalWrite(_pin_MOTOR_OUT2_1, LOW);
        } else if (_controlInput_1 < 0) {
            // CCW Rotation:
            analogWrite(_pin_PWM_1, abs(_controlInput_1));
            digitalWrite(_pin_MOTOR_OUT1_1, LOW);
            digitalWrite(_pin_MOTOR_OUT2_1, HIGH);
        }
    }

    // Motor 2 Control:
    if(_joy_stick_2_mode == 1) {
        if (_controlInput_2 > 0) {
            // CW Rotation:
            analogWrite(_pin_PWM_2, abs(_controlInput_2));
            digitalWrite(_pin_MOTOR_OUT1_2, HIGH);
            digitalWrite(_pin_MOTOR_OUT2_2, LOW);
        } else if (_controlInput_2 < 0) {
            // CCW Rotation:
            analogWrite(_pin_PWM_2, abs(_controlInput_2));
            digitalWrite(_pin_MOTOR_OUT1_2, LOW);
            digitalWrite(_pin_MOTOR_OUT2_2, HIGH);
        }
    }
    else{
        if (_error_2 < _ERROR_BOUND && _error_2 > -1 * _ERROR_BOUND) {
            // RESET INTEGRAL TERM:
            _int_error_2 = 0;
            // BREAK:
            analogWrite(_pin_PWM_2, 0);
            digitalWrite(_pin_MOTOR_OUT1_2, LOW);
            digitalWrite(_pin_MOTOR_OUT2_2, LOW);
        } else if (_controlInput_2 > 0) {
            // CW Rotation:
            analogWrite(_pin_PWM_2, abs(_controlInput_2));
            digitalWrite(_pin_MOTOR_OUT1_2, HIGH);
            digitalWrite(_pin_MOTOR_OUT2_2, LOW);
        } else if (_controlInput_2 < 0) {
            // CCW Rotation:
            analogWrite(_pin_PWM_2, abs(_controlInput_2));
            digitalWrite(_pin_MOTOR_OUT1_2, LOW);
            digitalWrite(_pin_MOTOR_OUT2_2, HIGH);
        }
    }
}

void JoyStickController::commandMode(float theta_desired, int motor_number) {
    // Control Law:
    if(motor_number == 1) {
        _prev_error_1 = _error_1;
        _error_1 = theta_desired -
                 (_motor_RESOLUTION_1 * encoder_1.read());
        _diff_error_1 = _error_1 - _prev_error_1;
        _int_error_1 = _int_error_1 + _error_1;
        _controlInput_1 = _Kp_1 * _error_1 + (_Kd_1 * _diff_error_1) * (_delay_1 != 0) + _Ki_1 * _int_error_1;
        _delay_1 = 1;

        // Saturate Control Input:
        if (_controlInput_1 > _SATURATION_LIMIT) {
            _controlInput_1 = _SATURATION_LIMIT;
        }
        if (_controlInput_1 < -1 * _SATURATION_LIMIT) {
            _controlInput_1 = -1 * _SATURATION_LIMIT;
        }

        if (_error_1 < _ERROR_BOUND && _error_1 > -1 * _ERROR_BOUND) {
            // RESET INTEGRAL TERM:
            _int_error_1 = 0;
            // BREAK:
            analogWrite(_pin_PWM_1, 0);
            digitalWrite(_pin_MOTOR_OUT1_1, LOW);
            digitalWrite(_pin_MOTOR_OUT2_1, LOW);
        } else if (_controlInput_1 > 0) {
            // CW Rotation:
            analogWrite(_pin_PWM_1, abs(_controlInput_1));
            digitalWrite(_pin_MOTOR_OUT1_1, HIGH);
            digitalWrite(_pin_MOTOR_OUT2_1, LOW);
        } else if (_controlInput_1 < 0) {
            // CCW Rotation:
            analogWrite(_pin_PWM_1, abs(_controlInput_1));
            digitalWrite(_pin_MOTOR_OUT1_1, LOW);
            digitalWrite(_pin_MOTOR_OUT2_1, HIGH);
        }
    }
    if(motor_number == 2) {
        _prev_error_2 = _error_2;
        _error_2 = theta_desired -
                 (_motor_RESOLUTION_2 * encoder_2.read());
        _diff_error_2 = _error_2 - _prev_error_2;
        _int_error_2 = _int_error_2 + _error_2;
        _controlInput_2 = _Kp_2 * _error_2 + (_Kd_2 * _diff_error_2) * (_delay_2 != 0) + _Ki_2 * _int_error_2;
        _delay_2 = 1;

        // Saturate Control Input:
        if (_controlInput_2 > _SATURATION_LIMIT) {
            _controlInput_2 = _SATURATION_LIMIT;
        }
        if (_controlInput_2 < -1 * _SATURATION_LIMIT) {
            _controlInput_2 = -1 * _SATURATION_LIMIT;
        }

        if (_error_2 < _ERROR_BOUND && _error_2 > -1 * _ERROR_BOUND) {
            // RESET INTEGRAL TERM:
            _int_error_2 = 0;
            // BREAK:
            analogWrite(_pin_PWM_2, 0);
            digitalWrite(_pin_MOTOR_OUT1_2, LOW);
            digitalWrite(_pin_MOTOR_OUT2_2, LOW);
        } else if (_controlInput_2 > 0) {
            // CW Rotation:
            analogWrite(_pin_PWM_2, abs(_controlInput_2));
            digitalWrite(_pin_MOTOR_OUT1_2, HIGH);
            digitalWrite(_pin_MOTOR_OUT2_2, LOW);
        } else if (_controlInput_2 < 0) {
            // CCW Rotation:
            analogWrite(_pin_PWM_2, abs(_controlInput_2));
            digitalWrite(_pin_MOTOR_OUT1_2, LOW);
            digitalWrite(_pin_MOTOR_OUT2_2, HIGH);
        }
    }
}

float JoyStickController::readEncoder(int motor_number) {
    if(motor_number == 1) {
        return encoder_1.read();
    }
    else if(motor_number == 2){
        return encoder_2.read();
    }
}

float JoyStickController::readAngle(int motor_number) {
    if(motor_number == 1) {
        return _motor_RESOLUTION_1 * encoder_1.read();
    }
    else if(motor_number == 2){
        return _motor_RESOLUTION_2 * encoder_2.read();
    }
}

float JoyStickController::readError(int motor_number) {
    if(motor_number == 1) {
        return _error_1;
    }
    else if(motor_number == 2){
        return _error_2;
    }
}

float JoyStickController::readDiffError(int motor_number) {
    if(motor_number == 1) {
        return _diff_error_1;
    }
    else if(motor_number == 2){
        return _diff_error_2;
    }
}

float JoyStickController::readIntError(int motor_number) {
    if(motor_number == 1) {
        return _int_error_1;
    }
    else if(motor_number == 2){
        return _int_error_2;
    }
}

float JoyStickController::readControlInput(int motor_number) {
    if(motor_number == 1) {
        return _controlInput_1;
    }
    else if(motor_number == 2){
        return _controlInput_2;
    }
}

float JoyStickController::readHallEncoder(int motor_number) {
    if(motor_number == 1) {
        return _hall_RESOLUTION_1 * analogRead(_pin_HALL_ENCODER_1);
    }
    else if(motor_number == 2){
        return _hall_RESOLUTION_2 * analogRead(_pin_HALL_ENCODER_2);
    }
}

int JoyStickController::readJoyStick(int motor_number) {
    if(motor_number == 1) {
        return analogRead(_pin_JOY_STICK_1);
    }
    else if(motor_number == 2){
        return analogRead(_pin_JOY_STICK_2);
    }
}

int JoyStickController::readJoyStickMode(int motor_number) {
    if(motor_number == 1) {
        return _joy_stick_1_mode;
    }
    else if(motor_number == 2){
        return _joy_stick_2_mode;
    }
}

int JoyStickController::readJoyStickInvert(int motor_number) {
    if(motor_number == 1) {
        return _joy_stick_1;
    }
    else if(motor_number == 2){
        return _joy_stick_2;
    }
}

float JoyStickController::readSaturationLimit(){
    return _SATURATION_LIMIT;
}

void JoyStickController::setNominalDeflection(int motor_number) {
    if(motor_number == 1) {
        _nominal_Deflection_1 = _hall_RESOLUTION_1 * analogRead(_pin_HALL_ENCODER_1);
    }
    else if(motor_number == 2){
        _nominal_Deflection_2 = _hall_RESOLUTION_2 * analogRead(_pin_HALL_ENCODER_2);
    }
}

void JoyStickController::gainScheduleCheck() {
    _old_Button_State_1 = _new_Button_State_1;
    _new_Button_State_1 = digitalRead(_pin_JOY_STICK_BUTTON_1);
    if(_old_Button_State_1 == 0 && _new_Button_State_1 == 1){
        _GAIN_SCHEDULE_ENABLE = !_GAIN_SCHEDULE_ENABLE;
    }
}

int JoyStickController::readGainSchedule() {
    return _GAIN_SCHEDULE_ENABLE;
}

void JoyStickController::modeSwitch() {
    _old_Button_State_2 = _new_Button_State_2;
    _new_Button_State_2 = digitalRead(_pin_JOY_STICK_BUTTON_2);
    if(_old_Button_State_2 == 0 && _new_Button_State_2 == 1){
        _MODE_SWITCH = !_MODE_SWITCH;
    }
}

bool JoyStickController::commandModeEnable() {
    return _MODE_SWITCH;
}