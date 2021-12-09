#include <Encoder.h>
#include <JoyStickController.h>

// Constants: (SPECIFIC TO OUR SETUP)
#define CPT 500
#define KNEE_GEAR_RATIO 138.0 / 5.0
#define ANKLE_GEAR_RATIO 23.0 / 4.0
#define PPR CPT * 4.0
#define KNEE_RESOLUTION 360.0 / (PPR * KNEE_GEAR_RATIO)
#define ANKLE_RESOLUTION 360.0 / (PPR * ANKLE_GEAR_RATIO)
#define HALL_ENCODER_RESOLUTION 360.0 / 4096.0

//For input through the Serial Monitor:
const unsigned int MAX_MESSAGE_LENGTH = 12;
int number;

//Motor Pins:
int kneePwm = 9;
int motorOutA1 = 5;
int motorOutA2 = 6;

int anklePwm = 10;
int motorOutB1 = 7;
int motorOutB2 = 8;

//Motor Encoder Pins:
int encoder1Apin = 18; // These pins are flipped to change direction
int encoder1Bpin = 19;
int index1Pin = 17;

int encoder2Apin = 20; // These pins are flipped to change direction
int encoder2Bpin = 21;
int index2Pin = 16;

//Hall Effect Encoder pins:
int hallencoder1_pin = A0;
int hallencoder2_pin = A1;

//Joy Stick Pins:
int pin_JOY_STICK_1 = A2;
int pin_JOY_STICK_2 = A3;
int pin_JOY_STICK_BUTTON_1 = 50;
int pin_JOY_STICK_BUTTON_2 = 52;

//PI Controller: (SPECIFIC TO OUR SETUP)
float Kp_1 = 4;
float Ki_1 = 0.05;
float Kp_2 = 3;
float Ki_2 = 0.05;
float K_joy_stick_1 = 0.15;
float K_joy_stick_2 = 0.25;
float theta_desired_1 = 0; // Initialize as 0
float theta_desired_2 = 0;

// Variable to flip between input to serial monitor for knee and ankle motor:
bool FLIP = 0;

//Controller: 
JoyStickController controller(kneePwm, motorOutA1, motorOutA2, encoder1Apin, encoder1Bpin, index1Pin, KNEE_RESOLUTION, hallencoder1_pin, HALL_ENCODER_RESOLUTION, pin_JOY_STICK_1, pin_JOY_STICK_BUTTON_1, Kp_1, Ki_1, 0, K_joy_stick_1, anklePwm, motorOutB1, motorOutB2, encoder2Apin, encoder2Bpin, index2Pin, ANKLE_RESOLUTION, hallencoder2_pin, HALL_ENCODER_RESOLUTION, pin_JOY_STICK_2, pin_JOY_STICK_BUTTON_2, Kp_2, Ki_2, 0, K_joy_stick_2);

void setup() {
  // Serial Initialization:
  Serial.begin(9600);
  controller.setNominalDeflection(1);
  controller.setNominalDeflection(2);
}

void loop() {

  // Command Desired Angle through Serial Monitor:
  while(Serial.available() > 0){
    //Create a place to hold the incoming message
    static char message[MAX_MESSAGE_LENGTH];
    static unsigned int message_pos = 0;

    //Read the next available byte in the serial receive buffer
    char inByte = Serial.read();

    //Message coming in (check not terminating character) and guard for over message size
    if(inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1)){
      //Add the incoming byte to our message
      message[message_pos] = inByte;
      message_pos++;
    }
    //Full message received...
    else{
      //Add null character to string
      message[message_pos] = '\0';

      //Or convert to integer and print
      number = atoi(message);
      Serial.println(number);

      //Reset for the next message
      message_pos = 0;

      FLIP = !FLIP;
    }
  }

  // Desired Angle Typecast:
  if(FLIP == 0){
    theta_desired_1 = float(number);
  }
  else if(FLIP == 1){
    theta_desired_2 = float(number);
  }

  controller.gainScheduleCheck();
  controller.modeSwitch();

  if(controller.commandModeEnable() == 0){
    controller.controllerMode();
  }
  else if(controller.commandModeEnable() == 1){
    controller.commandMode(theta_desired_1, 1);
    controller.commandMode(theta_desired_2, 2);
  }

// DEBUGGING:
//  // Encoder Test:
//  Serial.print(controller.readAngle(1));
//  Serial.print(" : ");
//  Serial.println(controller.readAngle(2));

//  // Hall Encoder Test:
//  Serial.print(controller.readHallEncoder(1));
//  Serial.print(" : ");
//  Serial.println(controller.readHallEncoder(2));

//  // Gain Schedule Check:
//  Serial.println(controller.readGainSchedule());
  
//  // Controller Command: MOTOR 1
//  Serial.print(controller.readJoyStick(1));
//  Serial.print(" : ");
//  Serial.print(controller.readJoyStickMode(1));
//  Serial.print(" : ");
//  Serial.print(controller.readAngle(1));
//  Serial.print(" : ");
//  Serial.print(controller.readError(1));
//  Serial.print(" : ");
//  Serial.print(controller.readIntError(1));
//  Serial.print(" : ");
//  Serial.println(controller.readControlInput(1));

//  // Controller Command: MOTOR 2
//  Serial.print(controller.readJoyStick(2));
//  Serial.print(" : ");
//  Serial.print(controller.readJoyStickMode(2));
//  Serial.print(" : ");
//  Serial.print(controller.readAngle(2));
//  Serial.print(" : ");
//  Serial.print(controller.readError(2));
//  Serial.print(" : ");
//  Serial.print(controller.readIntError(2));
//  Serial.print(" : ");
//  Serial.println(controller.readControlInput(2));
}
