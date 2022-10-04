/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\djsch                                            */
/*    Created:      Mon Aug 29 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftMotor            motor         1               
// RightMotor           motor         10              
// Controller1          controller                    
// StrafeMotor          motor         2               
// ArmMotor             motor         12              
// LowerIntakeMotor     motor         3               
// UpperIntakeMotor     motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace vex;

// constants
const double MAX_VOLTAGE_FWD = 12;
const double MAX_VOLTAGE_ROT = 12;
const double VOLTAGE_FEED_FWD = 1;
const double CHEESY_ROT_SENSITIVITY = 7; //6 out of 12v
const double MAX_VOLTAGE_ACCEL = 1;
const double MAX_VOLTAGE_ACCEL_STRAFE = 2;

enum ControlScheme {TWO_STICK_ARCADE, ONE_STICK_ARCADE, CHEESY_DRIVE};
ControlScheme controlScheme = TWO_STICK_ARCADE;

enum StickCurve {LINEAR, CUBIC};
StickCurve stickCurve = CUBIC;

void nextControlScheme() {
  switch (controlScheme) {
    case TWO_STICK_ARCADE: controlScheme = ONE_STICK_ARCADE; break;
    case ONE_STICK_ARCADE: controlScheme = CHEESY_DRIVE; break;
    case CHEESY_DRIVE: controlScheme = TWO_STICK_ARCADE; break;
  }
}

void nextStickCurve() {
  switch (stickCurve) {
    case LINEAR: stickCurve = CUBIC; break;
    case CUBIC: stickCurve = LINEAR; break;
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //constants 
  const auto leftYAxis = Controller1.Axis3;
  const auto leftXAxis = Controller1.Axis4;
  const auto rightYAxis = Controller1.Axis2;
  const auto rightXAxis = Controller1.Axis1;

  //state vars
  int n = 0;

  Controller1.ButtonA.pressed(nextControlScheme);
  Controller1.ButtonB.pressed(nextStickCurve);

  double lastLeftVoltage = 0;
  double lastRightVoltage = 0;
  double lastStrafeVoltage = 0;

  StrafeMotor.setStopping(brakeType::hold);
  LeftMotor.setStopping(brakeType::hold);
  RightMotor.setStopping(brakeType::hold);
  ArmMotor.setStopping(brakeType::hold);

  while (true) { //cubic curve on controls

      if (Controller1.ButtonL1.pressing()) {
        LowerIntakeMotor.spin(fwd, 200, rpm);
        UpperIntakeMotor.spin(fwd, 200, rpm);
      } else if (Controller1.ButtonL2.pressing()) {
        LowerIntakeMotor.spin(fwd, 200, rpm);
        UpperIntakeMotor.spin(directionType::rev, 200 , rpm);
      } else {
        LowerIntakeMotor.stop();
        UpperIntakeMotor.stop();
      }
      if (Controller1.ButtonX.pressing()) {
        ArmMotor.spin(fwd, 50, rpm);
      } else if (Controller1.ButtonY.pressing()) {
        ArmMotor.spin(fwd, -50, rpm);
      } else {
        ArmMotor.stop();
      }
    //measure sticks
    double leftX;
    double leftY;
    double rightX;
    double rightY;
    switch (stickCurve) {
      case LINEAR:
        leftX = leftXAxis.value() / 127 * sqrt(2);
        leftY = leftYAxis.value() / 127 * sqrt(2);
        rightX = rightXAxis.value() / 127 * sqrt(2);
        rightY = rightYAxis.value() / 127 * sqrt(2);
      case CUBIC:
        leftX = pow(((double) leftXAxis.value() * sqrt(2)) / 127, 3);
        leftY = pow(((double) leftYAxis.value() * sqrt(2)) / 127, 3);
        rightX = pow(((double) rightXAxis.value() * sqrt(2)) / 127, 3);
        rightY = pow(((double) rightYAxis.value() * sqrt(2)) / 127, 3);
    } 

    double leftMotorVoltage = 0;
    double rightMotorVoltage = 0;
    double strafeMotorVoltage = 0;

    switch (controlScheme) {
      case ONE_STICK_ARCADE:
        leftMotorVoltage = leftY * MAX_VOLTAGE_FWD - leftX * MAX_VOLTAGE_ROT;
        rightMotorVoltage = leftY * MAX_VOLTAGE_FWD + leftX * MAX_VOLTAGE_ROT;
        break;
      case TWO_STICK_ARCADE:
        leftMotorVoltage = leftY * MAX_VOLTAGE_FWD - rightX * MAX_VOLTAGE_ROT;
        rightMotorVoltage = leftY * MAX_VOLTAGE_FWD + rightX * MAX_VOLTAGE_ROT;
        strafeMotorVoltage = leftX * MAX_VOLTAGE_FWD; 
        break;
      case CHEESY_DRIVE: //constant curvature
        leftMotorVoltage = leftY * MAX_VOLTAGE_FWD - leftY * rightX * CHEESY_ROT_SENSITIVITY;
        rightMotorVoltage = leftY * MAX_VOLTAGE_FWD + leftY * rightX * CHEESY_ROT_SENSITIVITY;
    }
  
    //apply feed forward
    if (leftMotorVoltage > 0) leftMotorVoltage += VOLTAGE_FEED_FWD;
    if (leftMotorVoltage < 0) leftMotorVoltage -= VOLTAGE_FEED_FWD;
    if (rightMotorVoltage > 0) rightMotorVoltage += VOLTAGE_FEED_FWD;
    if (rightMotorVoltage < 0) rightMotorVoltage -= VOLTAGE_FEED_FWD;
    if (strafeMotorVoltage > 0) strafeMotorVoltage += VOLTAGE_FEED_FWD;
    if (strafeMotorVoltage < 0) strafeMotorVoltage -= VOLTAGE_FEED_FWD;
    //cap acceleration
    if (leftMotorVoltage > lastLeftVoltage + MAX_VOLTAGE_ACCEL) leftMotorVoltage = lastLeftVoltage + MAX_VOLTAGE_ACCEL;
    if (leftMotorVoltage < lastLeftVoltage - MAX_VOLTAGE_ACCEL) leftMotorVoltage = lastLeftVoltage - MAX_VOLTAGE_ACCEL;
    if (rightMotorVoltage > lastRightVoltage + MAX_VOLTAGE_ACCEL) rightMotorVoltage = lastRightVoltage + MAX_VOLTAGE_ACCEL;
    if (rightMotorVoltage < lastRightVoltage - MAX_VOLTAGE_ACCEL) rightMotorVoltage = lastRightVoltage - MAX_VOLTAGE_ACCEL;
    if (strafeMotorVoltage > lastStrafeVoltage + MAX_VOLTAGE_ACCEL_STRAFE) strafeMotorVoltage = lastStrafeVoltage + MAX_VOLTAGE_ACCEL_STRAFE;
    if (strafeMotorVoltage < lastStrafeVoltage - MAX_VOLTAGE_ACCEL_STRAFE) strafeMotorVoltage = lastStrafeVoltage - MAX_VOLTAGE_ACCEL_STRAFE;
    
    //cap voltages
    if (leftMotorVoltage > 12) leftMotorVoltage = 12;
    if (leftMotorVoltage < -12) leftMotorVoltage = -12; 
    if (rightMotorVoltage > 12) rightMotorVoltage = 12;
    if (rightMotorVoltage < -12) rightMotorVoltage = -12; 
    if (strafeMotorVoltage > 12) strafeMotorVoltage = 12;
    if (strafeMotorVoltage < -12) strafeMotorVoltage = -12; 
    
    //spin motors
    LeftMotor.spin(directionType::fwd, leftMotorVoltage, voltageUnits::volt);
    RightMotor.spin(directionType::fwd, rightMotorVoltage, voltageUnits::volt);
    StrafeMotor.spin(directionType::fwd, strafeMotorVoltage, voltageUnits::volt);

    if (leftMotorVoltage == 0) {
      LeftMotor.stop();
    }
    if (rightMotorVoltage == 0) {
      RightMotor.stop();
    }
    if (strafeMotorVoltage == 0) {
      StrafeMotor.stop();
    }

    
    //record voltages for next loop
    lastLeftVoltage = leftMotorVoltage;
    lastRightVoltage = rightMotorVoltage;
    lastStrafeVoltage = strafeMotorVoltage;
    
    if (n % 10 == 0) { //logging + io (every 200ms)
      Controller1.Screen.clearScreen();
      
      Controller1.Screen.setCursor(0,0);
      switch (controlScheme) {
        case ONE_STICK_ARCADE:
          Controller1.Screen.print("One Stick Arcade"); break;
        case TWO_STICK_ARCADE:
          Controller1.Screen.print("Two Stick Arcade"); break;
        case CHEESY_DRIVE: 
          Controller1.Screen.print("Cheesy Drive"); break;
      }

      Controller1.Screen.setCursor(3,0);
      switch (stickCurve) {
        case LINEAR:
          Controller1.Screen.print("Linear"); break;
        case CUBIC:
          Controller1.Screen.print("Cubic"); break;
      }
      // cout << "Left Volts: " << leftMotorVoltage << " Right Volts: " << rightMotorVoltage << " Strafe Volts: " << strafeMotorVoltage << endl;
      // cout << "Left RPM: " << LeftMotor.velocity(rpm) << " Right RPM: " << RightMotor.velocity(rpm) << endl;
    }

    task::sleep(20); //sleep 20ms
    n++;
  }
}