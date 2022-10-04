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
// LeftMotor            motor         1               
// RightMotor           motor         10              
// Controller1          controller                    
// StrafeMotor          motor         2               
// ArmMotor             motor         12              
// LowerIntakeMotor     motor         3               
// UpperIntakeMotor     motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

//custom motor wrappers
Motor leftMotor(LeftMotor, DRIVE_MOTOR_SETTINGS);
Motor rightMotor(RightMotor, DRIVE_MOTOR_SETTINGS);
Motor strafeMotor(StrafeMotor, DRIVE_MOTOR_SETTINGS);
Motor armMotor(ArmMotor, INTAKE_MOTOR_SETTINGS);
Motor lowerIntakeMotor(LowerIntakeMotor, INTAKE_MOTOR_SETTINGS);
Motor upperIntakeMotor(UpperIntakeMotor, INTAKE_MOTOR_SETTINGS);

//h-drive wrapper
HDrive hdrive(leftMotor, rightMotor, strafeMotor);
//constants
double MAX_VELOCITY_SLOW = 12; // in/s
double MAX_VELOCITY_FAST = 24; // in/s
double MAX_OMEGA_SLOW = .5;
double MAX_OMEGA_FAST = 5;

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
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    /* Intake Code */
    if (Controller1.ButtonL2.pressing()) {
      lowerIntakeMotor.drive(200);
      upperIntakeMotor.drive(200);
    } else if (Controller1.ButtonL1.pressing()) {
      lowerIntakeMotor.drive(200);
      upperIntakeMotor.drive(-200);
    } else {
      lowerIntakeMotor.drive(0);
      upperIntakeMotor.drive(0);
    }

    /* Drive Code */
    double v_max = Controller1.ButtonR2.pressing() ? MAX_VELOCITY_FAST : MAX_VELOCITY_SLOW;
    double omega_max = Controller1.ButtonR2.pressing() ? MAX_OMEGA_FAST : MAX_OMEGA_SLOW;

    hdrive.setDriveVelocities(Controller1.Axis3.position(percent) * v_max / 100, // v_x
                              Controller1.Axis4.position(percent) * v_max / 100, // v_y
                              Controller1.Axis1.position(percent) * omega_max / 100); // omega

    /* call loop functions */
    hdrive.loop();
    lowerIntakeMotor.loop();
    upperIntakeMotor.loop();

    wait(20, msec); // Sleep the task for a short amount of time to
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
