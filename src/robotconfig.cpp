#include "vex.h"
using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here
// VEXcode device constructors
controller controller1 = controller(primary);

// Six Motor Blue Gear Drive - For Six Motor Drive Trains
//motor LeftFrontMotor(PORT3, ratio6_1, false); 
//motor LeftStackMotor(PORT2, ratio6_1, false); 
//motor LeftBackMotor(PORT6, ratio6_1, true);
//motor RightStackMotor(PORT11, ratio6_1, true); 
//motor RightFrontMotor(PORT1, ratio6_1, true); 
//motor RightBackMotor(PORT12, ratio6_1, false); 

// Motor Groups 
//motor_group LeftMotorGroup = motor_group(LeftFrontMotor, LeftBackMotor, LeftStackMotor);
//motor_group RightMotorGroup = motor_group(RightFrontMotor, RightBackMotor, RightStackMotor);

//Two Motor Ghost DriveTrain
motor LeftMotor(PORT4, ratio18_1, false); 
motor RightMotor(PORT14, ratio18_1, true); 

// Motor Groups 
motor_group LeftMotorGroup = motor_group(LeftMotor);
motor_group RightMotorGroup = motor_group(RightMotor);


//Sensors
inertial inert = inertial(PORT5);