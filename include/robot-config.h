using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LeftMotor;
extern motor RightMotor;
extern controller Controller1;
extern motor StrafeMotor;
extern motor LowerIntakeMotor;
extern motor UpperIntakeMotor;
extern motor_group ArmMotors;
extern sonar RangeFinder;
extern line RightLineTracker;
extern line LeftLineTracker;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );