#ifndef JOYSTICK_JOYSTICKCONTROLLER_H
#define JOYSTICK_JOYSTICKCONTROLLER_H

#include "Arduino.h"
#include <Encoder.h>

class JoyStickController {
public:
    JoyStickController(int pin_PWM_1, int pin_MOTOR_OUT1_1, int pin_MOTOR_OUT2_1, int pin_ENCODER_A_1,
                       int pin_ENCODER_B_1, int pin_INDEX_1, float motor_RESOLUTION_1, int pin_HALL_ENCODER_1,
                       float hall_RESOLUTION_1, int pin_JOY_STICK_1, int pin_JOY_STICK_BUTTON_1, float Kp_1, float Ki_1, float Kd_1, float K_joy_stick_1,
                       int pin_PWM_2, int pin_MOTOR_OUT1_2, int pin_MOTOR_OUT2_2, int pin_ENCODER_A_2,
                       int pin_ENCODER_B_2, int pin_INDEX_2, float motor_RESOLUTION_2, int pin_HALL_ENCODER_2,
                       float hall_RESOLUTION_2, int pin_JOY_STICK_2, int pin_JOY_STICK_BUTTON_2, float Kp_2, float Ki_2, float Kd_2, float K_joy_stick_2);
    
    void controllerMode();
    void commandMode(float theta_desired, int motor_number);

    float readEncoder(int motor_number);
    float readAngle(int motor_number);
    float readError(int motor_number);
    float readDiffError(int motor_number);
    float readIntError(int motor_number);
    float readControlInput(int motor_number);
    float readHallEncoder(int motor_number);
    int readJoyStick(int motor_number);
    int readJoyStickMode(int motor_number);
    int readJoyStickInvert(int motor_number);
    float readSaturationLimit();
    void setNominalDeflection(int motor_number);
    void gainScheduleCheck();
    int readGainSchedule();
    void modeSwitch();
    bool commandModeEnable();

private:
    // Motor Pins / Constants: MOTOR 1
    int _pin_PWM_1;
    int _pin_MOTOR_OUT1_1;
    int _pin_MOTOR_OUT2_1;

    // Motor Pins / Constants: MOTOR 2
    int _pin_PWM_2;
    int _pin_MOTOR_OUT1_2;
    int _pin_MOTOR_OUT2_2;

    // Encoder Pins / Constants: MOTOR 1
    int _pin_ENCODER_A_1;
    int _pin_ENCODER_B_1;
    int _pin_INDEX_1;
    int _pin_HALL_ENCODER_1;

    // Encoder Pins / Constants: MOTOR 2
    int _pin_ENCODER_A_2;
    int _pin_ENCODER_B_2;
    int _pin_INDEX_2;
    int _pin_HALL_ENCODER_2;

    // Joy Stick Pins / Variables:
    int _pin_JOY_STICK_1;
    int _pin_JOY_STICK_2;
    int _pin_JOY_STICK_BUTTON_1;
    int _pin_JOY_STICK_BUTTON_2;
    float _joy_stick_1;
    float _joy_stick_2;
    int _joy_stick_1_mode;
    int _joy_stick_2_mode;

    // Resolution of Encoder: MOTOR 1
    float _motor_RESOLUTION_1;
    float _hall_RESOLUTION_1;

    // Resolution of Encoder: MOTOR 2
    float _motor_RESOLUTION_2;
    float _hall_RESOLUTION_2;

    // Control Law Gains:
    float _Kp_1;
    float _Ki_1;
    float _Kd_1;
    float _K_joy_stick_1;
    float _Kp_2;
    float _Ki_2;
    float _Kd_2;
    float _K_joy_stick_2;

    // Encoder Classes:
    Encoder encoder_1;
    Encoder encoder_2;

    // Control Variables:
    float _error_1;
    float _prev_error_1;
    float _diff_error_1;
    float _int_error_1;
    float _controlInput_1;
    float _error_2;
    float _prev_error_2;
    float _diff_error_2;
    float _int_error_2;
    float _controlInput_2;
    int _delay_1;
    int _delay_2;
    float _theta_hold_1;
    float _theta_hold_2;

    // Bound Constants:
    float _ERROR_BOUND;
    float _NOMINAL_JOY_STICK;
    float _DEAD_ZONE;
    float _SATURATION_LIMIT;

    // Gain Schedule Variables:
    bool _GAIN_SCHEDULE_ENABLE;
    float _JOY_STICK_GAIN_1;
    float _JOY_STICK_GAIN_2;
    float _nominal_Deflection_1;
    float _nominal_Deflection_2;
    bool _new_Button_State_1;
    bool _old_Button_State_1;

    // Mode Switch:
    bool _MODE_SWITCH;
    bool _old_Button_State_2;
    bool _new_Button_State_2;
};

#endif //JOYSTICK_JOYSTICKCONTROLLER_H
