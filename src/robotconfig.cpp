#include "vex.h"
using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen

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
motor RightFront = motor(PORT18, ratio6_1, false); //done
motor RightMiddle = motor(PORT19, ratio6_1, true); //done
motor RightRear = motor(PORT21, ratio6_1, false); //done
motor LeftFront = motor(PORT13, ratio6_1, true); //done
motor LeftMiddle = motor(PORT12, ratio6_1, false); //done
motor LeftRear = motor(PORT11, ratio6_1, true); //done
inertial inert = inertial(PORT5);

motor FrontIntakeRight = motor(PORT17, ratio6_1, true); //done
motor FrontIntakeLeft = motor(PORT14, true); //done
motor Outtake = motor(PORT9, false); //done

motor_group LeftMotorGroup = motor_group(LeftFront, LeftMiddle, LeftRear);
motor_group RightMotorGroup = motor_group(RightFront, RightMiddle, RightRear);
motor_group IntakeFrontGroup = motor_group(FrontIntakeRight, FrontIntakeLeft);
motor_group AllMotorGroup = motor_group(LeftFront, RightFront, LeftMiddle, RightMiddle, LeftRear, RightRear);
motor_group TrulyAllMotorGroup = motor_group(LeftFront, RightFront, LeftMiddle, RightMiddle, LeftRear, RightRear, FrontIntakeRight, FrontIntakeLeft, Outtake);
// Motor Groups 
//motor_group LeftMotorGroup = motor_group(LeftFrontMotor, LeftBackMotor, LeftStackMotor);
//motor_group RightMotorGroup = motor_group(RightFrontMotor, RightBackMotor, RightStackMotor);
