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
// LowerIntakeMotor     motor         3               
// UpperIntakeMotor     motor         4               
// ArmMotors            motor_group   11, 12          
// RangeFinder          sonar         A, B            
// RightLineTracker     line          C               
// LeftLineTracker      line          D               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>
#include <algorithm>
#include <cmath>

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

void driveIntoWall(double velocity) { //rpm
  //drive into the wall at velocity until velocity drops below velocity
  LeftMotor.spin(directionType::fwd, velocity, velocityUnits::rpm);
  RightMotor.spin(directionType::fwd, velocity, velocityUnits::rpm);
  wait(500, msec);

  while (LeftMotor.velocity(velocityUnits::rpm) > velocity / 4) {
    wait(100, msec);
  }

  LeftMotor.stop();
  RightMotor.stop();
}

void lineSquare(double vtowards, double vaway) {
  LeftMotor.setStopping(coast);
  RightMotor.setStopping(coast);
  bool leftDone = false;
  bool rightDone = false;
  LeftMotor.spin(directionType::fwd, vtowards, velocityUnits::rpm);
  RightMotor.spin(directionType::fwd, vtowards, velocityUnits::rpm);
  cout << "waiting for towards..." << endl;
  cout << "left: " << LeftLineTracker.reflectivity(percentUnits::pct) << ", right: " << RightLineTracker.reflectivity(percentUnits::pct) << endl;
  while (!(leftDone && rightDone)) {
    cout << "left: " << LeftLineTracker.reflectivity(percentUnits::pct) << ", right: " << RightLineTracker.reflectivity(percentUnits::pct) << endl;
    if (RightLineTracker.reflectivity(percentUnits::pct) > 20) {
      rightDone = true;
      RightMotor.stop();
    }
    if (LeftLineTracker.reflectivity(percentUnits::pct) > 20) {
      leftDone = true;
      LeftMotor.stop();
    }
    wait(20, msec);
  }
  
  //go back
  cout << "waiting for away..." << endl;
  LeftMotor.spin(directionType::fwd, vaway, velocityUnits::rpm);
  RightMotor.spin(directionType::fwd, vaway, velocityUnits::rpm);
  while (!leftDone && !rightDone) {
    if (RightLineTracker.reflectivity(percentUnits::pct) < 15) {
      rightDone = true;
      RightMotor.stop();
    }
    if (LeftLineTracker.reflectivity(percentUnits::pct) < 15) {
      leftDone = true;
      LeftMotor.stop();
    }
    wait(20, msec);
  }
  cout << "done" << endl;

  LeftMotor.setStopping(brake);
  RightMotor.setStopping(brake);
}

void pidDistance(double dist, double p, double ff) {
  double rangeDist = RangeFinder.distance(distanceUnits::cm);
  cout << "dist: " << rangeDist << ", err: " << abs(rangeDist - dist) << endl;
  int succ = 0; //# of successe
  while ( succ < 4) {
    if (abs(rangeDist - dist) > 2.5) {
      succ = 0;
    } else {
      succ++;
    }
    cout << "dist: " << rangeDist << ", err: " << abs(rangeDist - dist) << endl;
    double err = rangeDist - dist;
    StrafeMotor.spin(directionType::rev, err*p, velocityUnits::rpm);
    wait(20, msec);
    rangeDist = RangeFinder.distance(distanceUnits::cm);
  }
  cout << "PID done, dist: " << rangeDist << endl;
  StrafeMotor.stop();
}

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
  ArmMotors.setStopping(brakeType::hold);
  UpperIntakeMotor.setStopping(brakeType::hold);
  
  //prevent burnout on drive
  LeftMotor.setMaxTorque(90, percentUnits::pct);
  RightMotor.setMaxTorque(90, percentUnits::pct);
  StrafeMotor.setMaxTorque(90, percentUnits::pct);
  LeftMotor.setTimeout(3, timeUnits::sec);
  RightMotor.setTimeout(3, timeUnits::sec);
  StrafeMotor.setTimeout(3, timeUnits::sec);


  ArmMotors.spin(directionType::fwd, -20, velocityUnits::rpm); //hit motors into hard stop
  wait(1000, msec);
  ArmMotors.resetRotation();
  ArmMotors.spinToPosition(0.4, rotationUnits::rev, 45.5, velocityUnits::rpm);

  

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
    ArmMotors.stop();
    ArmMotors.spinToPosition(3.3, rotationUnits::rev, 80, velocityUnits::rpm);
  });
  Controller1.ButtonA.pressed([] () {
    ArmMotors.stop();
    ArmMotors.spinToPosition(1.9, rotationUnits::rev, 80, velocityUnits::rpm);
  });
  Controller1.ButtonB.pressed([] () {
    ArmMotors.stop();
    ArmMotors.spinToPosition(0.3, rotationUnits::rev, 80, velocityUnits::rpm);
  });
  Controller1.ButtonY.pressed([] () {
    ArmMotors.stop();
    cout << "Arm Motor Stopped (Pos: " << ArmMotors.position(rotationUnits::rev) << ")" << endl;
  });

  bool manualArm = false;

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    /* testing */

    if (Controller1.ButtonLeft.pressing()) {
      pidDistance(100, -5, 20);
    }

    if (Controller1.ButtonDown.pressing()) {
      ArmMotors.spinToPosition(.4, rotationUnits::rev, 50, velocityUnits::rpm, true);
      LowerIntakeMotor.spin(directionType::fwd, 200, velocityUnits::rpm);
      UpperIntakeMotor.spinToPosition(0, rotationUnits::deg, 180, velocityUnits::rpm, false);
      driveIntoWall(80);
      wait(200,msec);
      ArmMotors.spinToPosition(.5, rotationUnits::rev, 50, velocityUnits::rpm, true);
      //back + strafe
      LeftMotor.spinFor(fwd, -300, rotationUnits::deg, 80, velocityUnits::rpm, false);
      StrafeMotor.spinFor(fwd, -100, rotationUnits::deg, 80, velocityUnits::rpm, false);
      RightMotor.spinFor(fwd, -300, rotationUnits::deg, 80, velocityUnits::rpm, true);
      
      wait(500, msec);
      //spin + lift
      ArmMotors.spinToPosition(3, rotationUnits::rev, 80, velocityUnits::rpm, false);
      LeftMotor.spinFor(fwd, -280, rotationUnits::deg, 50, velocityUnits::rpm, false);
      RightMotor.spinFor(fwd, 280, rotationUnits::deg, 50, velocityUnits::rpm, true);

      wait(500,msec);

      //square + turn
      lineSquare(25, -25);

      wait(500,msec);

      //bang + deliver
      driveIntoWall(150);
      wait(500, msec);
      LeftMotor.spinFor(fwd, -100, rotationUnits::deg, 50, velocityUnits::rpm, false);
      RightMotor.spinFor(fwd, -100, rotationUnits::deg, 50, velocityUnits::rpm, true);
      wait(500, msec);
      pidDistance(12, -5, -20);
      driveIntoWall(80);
      for (int i = 0; i < 3; i++) {
        LowerIntakeMotor.spin(directionType::fwd, 200, velocityUnits::rpm);
        UpperIntakeMotor.spinToPosition(-120, rotationUnits::deg, 200, velocityUnits::rpm, false);
        wait(400, msec);
        LowerIntakeMotor.stop();
        UpperIntakeMotor.spinToPosition(0, rotationUnits::deg, 200, velocityUnits::rpm, false);
        wait(200, msec);
      }

      //strafe + back up
      LeftMotor.spinFor(fwd, -100, rotationUnits::deg, 50, velocityUnits::rpm, false);
      RightMotor.spinFor(fwd, -100, rotationUnits::deg, 50, velocityUnits::rpm, true);

      StrafeMotor.spinFor(fwd, -200, rotationUnits::deg, 50, velocityUnits::rpm, true);
      lineSquare(-100,25);
      lineSquare(-25,25);
      
      LeftMotor.spinFor(fwd, -150, rotationUnits::deg, 50, velocityUnits::rpm, false);
      RightMotor.spinFor(fwd, -150, rotationUnits::deg, 50, velocityUnits::rpm, true);

      //strafe to ramp + arm down
      RightMotor.spinFor(fwd, 25, rotationUnits::deg, 50, velocityUnits::rpm, false); //spin so that the strafe doesnt
      LeftMotor.spinFor(fwd, -25, rotationUnits::deg, 50, velocityUnits::rpm, true);
      
      pidDistance(125, -5, -20);

      ArmMotors.spinToPosition(0.3, rotationUnits::rev, 80, velocityUnits::rpm, true);

      driveIntoWall(-80);

      LeftMotor.setMaxTorque(100, percent);
      RightMotor.setMaxTorque(100, percent);
      LeftMotor.spin(fwd, 12, voltageUnits::volt);
      RightMotor.spin(fwd, 12, voltageUnits::volt);

      wait(6000, msec);

    }


    /* Intake Code */
    if (Controller1.ButtonL2.pressing()) {
      LowerIntakeMotor.spin(directionType::fwd, 200, velocityUnits::rpm);
      UpperIntakeMotor.spinToPosition(0, rotationUnits::deg, 200, velocityUnits::rpm, false);
    } else if (Controller1.ButtonL1.pressing()) {
      LowerIntakeMotor.spin(directionType::fwd, 200, velocityUnits::rpm);
      UpperIntakeMotor.spinToPosition(-120, rotationUnits::deg, 200, velocityUnits::rpm, false);
    } else {
      LowerIntakeMotor.stop();
      UpperIntakeMotor.spinToPosition(0, rotationUnits::deg, 200, velocityUnits::rpm, false);
    }

    /* Lift Code */
    if (Controller1.ButtonY.pressing()) {
      manualArm = true;
      ArmMotors.spin(directionType::fwd, Controller1.Axis2.value() / 127 * 45.5, velocityUnits::rpm);
      continue; //no driving
    } else if (manualArm) {
      ArmMotors.stop();
      manualArm = false;
    }

    /* Real Turbo Mode */
    if (Controller1.ButtonR1.pressing()) {
      LeftMotor.setMaxTorque(100, percent);
      RightMotor.setMaxTorque(100, percent);
      LeftMotor.spin(fwd, 12, voltageUnits::volt);
      RightMotor.spin(fwd, 12, voltageUnits::volt);
      continue;
    }
    /* Drive Code */
    double v_max = Controller1.ButtonR2.pressing() ? MAX_VELOCITY_FAST : MAX_VELOCITY_SLOW;
    double omega_max = Controller1.ButtonR2.pressing() ? MAX_OMEGA_FAST : MAX_OMEGA_SLOW;
    
    hdrive.setDriveVelocities(Controller1.Axis3.value() * v_max / 127, // v_x
                              Controller1.Axis4.value() * v_max / 127, // v_y
                              Controller1.Axis1.value() * omega_max / 127); // omega

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