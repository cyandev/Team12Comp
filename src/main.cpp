/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftMotor            motor         10              
// RightMotor           motor         1               
// Controller1          controller                    
// StrafeMotor          motor         2               
// ArmMotor             motor         12              
// LowerIntakeMotor     motor         3               
// UpperIntakeMotor     motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>

using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

//constants
double MAX_VELOCITY_SLOW = 12; // in/s
double MAX_VELOCITY_FAST = 24; // in/s
double MAX_OMEGA_SLOW = .5;
double MAX_OMEGA_FAST = 1;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  StrafeMotor.setStopping(brakeType::hold);
  LeftMotor.setStopping(brakeType::hold);
  RightMotor.setStopping(brakeType::hold);
  ArmMotor.setStopping(brakeType::hold);

  //hit the arm of the hard stop
  ArmMotor.spinToPosition(1, rotationUnits::rev, 45.5, velocityUnits::rpm, true);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  cout << "Auto" << endl;
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  cout << "Teleop" << endl;
  // User control code here, inside the loop

  // Controller Control Scheme:
  // TODO Axis3, v_x (continuous)
  // TODO Axis4, v_y (continuous)
  // TODO Axis1, omega (continuous)
  // TODO ButtonX, red arm height (event)
  // TODO ButtonA, blue arm height (event)
  // TODO ButtonB, ground arm height (event)
  // TODO ButtonY + Axis2, v_arm (continuous)
  // TODO L2, intake (continuous)
  // TODO L1, outtake (continuous)
  // TODO R2, drive fast (continuous)
  HDrive hdrive(LeftMotor, RightMotor, StrafeMotor);

  Controller1.ButtonX.pressed([] () {
    ArmMotor.stop();
    ArmMotor.spinToPosition(3.1, rotationUnits::rev, 45.5, velocityUnits::rpm);
  });
  Controller1.ButtonA.pressed([] () {
    ArmMotor.stop();
    ArmMotor.spinToPosition(1, rotationUnits::rev, 45.5, velocityUnits::rpm);
  });
  Controller1.ButtonB.pressed([] () {
    ArmMotor.stop();
    ArmMotor.spinToPosition(.5, rotationUnits::rev, 45.5, velocityUnits::rpm);
  });
  Controller1.ButtonY.pressed([] () {
    ArmMotor.stop();
    cout << "Arm Motor Stopped (Pos: " << ArmMotor.position(rotationUnits::rev) << ")" << endl;
  });
  bool manualArm = false;

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    /* Intake Code */
    if (Controller1.ButtonL2.pressing()) {
      cout << "INTAKEE" << endl;
      LowerIntakeMotor.spin(directionType::fwd, 200, velocityUnits::rpm);
      UpperIntakeMotor.spin(directionType::fwd, 200, velocityUnits::rpm);
    } else if (Controller1.ButtonL1.pressing()) {
      LowerIntakeMotor.spin(directionType::fwd, 200, velocityUnits::rpm);
      UpperIntakeMotor.spin(directionType::fwd, -200, velocityUnits::rpm);
    } else {
      LowerIntakeMotor.stop();
      UpperIntakeMotor.stop();
    }

    /* Lift Code */
    if (Controller1.ButtonY.pressing()) {
      cout << "MANUAL ARM!" << endl;
      manualArm = true;
      ArmMotor.spin(directionType::fwd, Controller1.Axis2.value() / 127 * 45.5, velocityUnits::rpm);
      continue; //no driving
    } else if (manualArm) {
      ArmMotor.stop();
      manualArm = false;
    }
    /* Drive Code */
    double v_max = Controller1.ButtonR2.pressing() ? MAX_VELOCITY_FAST : MAX_VELOCITY_SLOW;
    double omega_max = Controller1.ButtonR2.pressing() ? MAX_OMEGA_FAST : MAX_OMEGA_SLOW;
    
    hdrive.setDriveVelocities(Controller1.Axis3.value() * v_max / 127, // v_x
                              Controller1.Axis4.value() * v_max / 127, // v_y
                              Controller1.Axis1.value() * omega_max / 127); // omega

    wait(100, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.

  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
