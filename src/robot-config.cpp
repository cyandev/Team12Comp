#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftMotor = motor(PORT10, ratio18_1, true);
motor RightMotor = motor(PORT1, ratio18_1, false);
controller Controller1 = controller(primary);
motor StrafeMotor = motor(PORT2, ratio18_1, false);
motor LowerIntakeMotor = motor(PORT3, ratio18_1, true);
motor UpperIntakeMotor = motor(PORT4, ratio18_1, false);
motor ArmMotorsMotorA = motor(PORT11, ratio18_1, true);
motor ArmMotorsMotorB = motor(PORT12, ratio18_1, false);
motor_group ArmMotors = motor_group(ArmMotorsMotorA, ArmMotorsMotorB);
sonar RangeFinder = sonar(Brain.ThreeWirePort.A);
line RightLineTracker = line(Brain.ThreeWirePort.C);
line LeftLineTracker = line(Brain.ThreeWirePort.D);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}