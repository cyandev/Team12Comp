using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LeftMotor;
extern motor RightMotor;
extern controller Controller1;
extern motor StrafeMotor;
extern motor ArmMotor;
extern motor LowerIntakeMotor;
extern motor UpperIntakeMotor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );