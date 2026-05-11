//   This program is template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/rcmgames/RCMv2
//   for information see this page: https://github.com/RCMgames

/**
UNCOMMENT ONE OF THE FOLLOWING LINES DEPENDING ON WHAT HARDWARE YOU ARE USING
Remember to also choose the "environment" for your microcontroller in PlatformIO
*/
// #define RCM_HARDWARE_VERSION RCM_ORIGINAL // versions 1, 2, 3, and 3.1 of the original RCM hardware // https://github.com/RCMgames/RCM_hardware_documentation_and_user_guide
// #define RCM_HARDWARE_VERSION RCM_4_V1 // version 1 of the RCM 4 // https://github.com/RCMgames/RCM-Hardware-V4
// #define RCM_HARDWARE_VERSION RCM_BYTE_V2 // version 2 of the RCM BYTE // https://github.com/RCMgames/RCM-Hardware-BYTE
#define RCM_HARDWARE_VERSION RCM_NIBBLE_V1 // version 1 of the RCM Nibble // https://github.com/RCMgames/RCM-Hardware-Nibble
// #define RCM_HARDWARE_VERSION RCM_D1_V1 // version 1 of the RCM D1 // https://github.com/RCMgames/RCM-Hardware-D1
// #define RCM_HARDWARE_VERSION ALFREDO_NOU2_NO_VOLTAGE_MONITOR // voltageComp will always report 10 volts https://www.alfredosys.com/products/alfredo-nou2/
// #define RCM_HARDWARE_VERSION ALFREDO_NOU2_WITH_VOLTAGE_MONITOR // modified to add resistors VIN-30k-D36-10k-GND https://www.alfredosys.com/products/alfredo-nou2/
// #define RCM_HARDWARE_VERSION ALFREDO_NOU3 // https://www.alfredosys.com/products/alfredo-nou3/

/**
uncomment one of the following lines depending on which communication method you want to use
*/
#define RCM_COMM_METHOD RCM_COMM_EWD // use the normal communication method for RCM robots

#include "rcm.h" //defines pins

// set up motors and anything else you need here
// See this page for how to set up servos and motors for each type of RCM board:
// https://github.com/RCMgames/useful-code/tree/main/boards
// See this page for information about how to set up a robot's drivetrain using the JMotor library
// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain

/**
Bits of code that might be helpful when starting a program with an original RCM board
https://github.com/RCMgames/RCM-Hardware-Nibble
// from https://github.com/RCMgames/useful-code/tree/main/boards
*/

#include "ICM20948_helper.h"

JEncoderQuadratureAttachInterrupt encoder1 = JEncoderQuadratureAttachInterrupt(port1Pin, port2Pin, 1.0/2550.0, false);
JEncoderQuadratureAttachInterrupt encoder2 = JEncoderQuadratureAttachInterrupt(port3Pin, port4Pin, 1.0/2550.0,true);

// TODO: do floats cause problems if the wheels have turned many times?
float local_left_pos = 0;
float local_right_pos = 0;
float local_left_vel = 0;
float local_right_vel = 0;

// all the motor drivers
JMotorDriverTMC7300 motor1Driver = JMotorDriverTMC7300(portA);
JMotorDriverTMC7300 motor2Driver = JMotorDriverTMC7300(portB);
JMotorDriverTMC7300 motor3Driver = JMotorDriverTMC7300(portC);
JMotorDriverTMC7300 motor4Driver = JMotorDriverTMC7300(portD);

float local_left_motor_power = 0;
float local_right_motor_power = 0;

int32_t controller_micros = 0;
float controller_batteryVoltage = 0;
boolean controller_button = false;

float remote_left_pos = 0;
float remote_right_pos = 0;
float remote_left_vel = 0;
float remote_right_vel = 0;

void Enabled()
{
    // WRITE CONTROLS HERE!!
    /*
    inputs:
    * enabled (true if car is connected)
    * local_left_pos, local_right_pos (wheel positions)
    * local_left_vel, local_right_vel (wheel velocities)
    *
    * imu (type imu. and see what variables are in the autocomplete)
    *
    outputs:
    * local_left_motor_power, local_right_motor_power, motor3Val, motor4Val (floats between -1 and 1 that control the motors)
    *
    *
    */

    // position to position
    // local_left_motor_power = (remote_left_pos - local_left_pos) * 1.0;
    // local_right_motor_power = (remote_right_pos - local_right_pos) * 1.0;

    // (springy) position to velocity
    local_left_motor_power = remote_left_pos;
    local_right_motor_power = remote_right_pos;

    // debugging controller
    // local_left_motor_power = 0;
    // local_right_motor_power = 0;

    RSLcolor = (controller_button ? CRGB(255, 255, 255) : CRGB(250, 45, 0));

    // set motors
    motor1Driver.set(-local_left_motor_power);
    motor2Driver.set(-local_right_motor_power);
    motor3Driver.set(local_left_motor_power);
    motor4Driver.set(local_right_motor_power);
}

void Enable()
{
    // turn on outputs
    motor1Driver.enable();
    motor2Driver.enable();
    motor3Driver.enable();
    motor4Driver.enable();
}

void Disable()
{
    // turn off outputs
    motor1Driver.disable();
    motor2Driver.disable();
    motor3Driver.disable();
    motor4Driver.disable();
}

jENCODER_MAKE_ISRS_MACRO(encoder1)
jENCODER_MAKE_ISRS_MACRO(encoder2)

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
    nibbleSetupImu();
    encoder1.setUpInterrupts(encoder1_jENCODER_ISR_A, encoder1_jENCODER_ISR_B);
    encoder2.setUpInterrupts(encoder2_jENCODER_ISR_A, encoder2_jENCODER_ISR_B);
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you don't also need to call leftMotorController.run())
    runIMU();
    encoder1.run();
    encoder2.run();

    local_left_pos = encoder1.getPos();
    local_right_pos = encoder2.getPos();
    local_left_vel = encoder1.getVel();
    local_right_vel = encoder2.getVel();

    Serial.print(local_left_pos);
    Serial.print(", ");
    Serial.print(local_left_vel);
    Serial.print(", ");
    Serial.print(local_right_pos);
    Serial.print(", ");
    Serial.print(local_right_vel);
    Serial.print(", ");
    Serial.print(remote_left_pos);
    Serial.print(", ");
    Serial.println(remote_right_pos);

    // delay(1);
}

#if RCM_COMM_METHOD == RCM_COMM_EWD
void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
    controller_batteryVoltage = EWD::recvFl();
    controller_micros = EWD::recvIn();
    controller_button = EWD::recvBl();
    remote_left_pos = EWD::recvFl();
    remote_right_pos = EWD::recvFl();
    remote_left_vel = EWD::recvFl();
    remote_right_vel = EWD::recvFl();
}
void WifiDataToSend()
{
    EWD::sendBl(true); // enabled
    EWD::sendFl(voltageComp.getSupplyVoltage());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
    EWD::sendIn(micros());
    EWD::sendBl(digitalRead(0) == 0);
    EWD::sendFl(local_left_pos);
    EWD::sendFl(local_right_pos);
    EWD::sendFl(local_left_vel);
    EWD::sendFl(local_right_vel);
}

void configWifi()
{
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "BEJM_controller";
    EWD::routerPassword = "hapticsBEJM";
    EWD::routerPort = 25210;
    EWD::communicateWithIP = "192.168.4.1";
    EWD::resendTimeout = 50;
    EWD::signalLossTimeout = 150;
}
#endif

#include "rcmutil.h"
