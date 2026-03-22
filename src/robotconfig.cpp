#include "vex.h"
using namespace vex;
controller controller1 = controller(primary);
//DEFINED Drivetrain
motor RightFront = motor(PORT18, ratio6_1, false);
motor RightMiddle = motor(PORT19, ratio6_1, true);
motor RightRear = motor(PORT21, ratio6_1, false);
motor LeftFront = motor(PORT13, ratio6_1, true);
motor LeftMiddle = motor(PORT12, ratio6_1, false);
motor LeftRear = motor(PORT11, ratio6_1, true);
inertial inert = inertial(PORT5);

//DEFINEDIn/Outtake
motor FrontIntakeRight = motor(PORT17, ratio6_1, true);
motor FrontIntakeLeft = motor(PORT14, true);
motor Outtake = motor(PORT9, false);

//DEFINED Motor Groups
motor_group LeftMotorGroup = motor_group(LeftFront, LeftMiddle, LeftRear);
motor_group RightMotorGroup = motor_group(RightFront, RightMiddle, RightRear);
motor_group IntakeFrontGroup = motor_group(FrontIntakeRight, FrontIntakeLeft);
motor_group AllMotorGroup = motor_group(LeftFront, RightFront, LeftMiddle, RightMiddle, LeftRear, RightRear);
motor_group TrulyAllMotorGroup = motor_group(LeftFront, RightFront, LeftMiddle, RightMiddle, LeftRear, RightRear, FrontIntakeRight, FrontIntakeLeft, Outtake);

/*Ghost Bot Definitions

motor RightFront = motor(PORT18, ratio6_1, true);
motor RightMiddle = motor(PORT19, ratio6_1, true);
motor RightRear = motor(PORT21, ratio6_1, false);
motor LeftFront = motor(PORT11, ratio6_1, false);
motor LeftMiddle = motor(PORT12, ratio6_1, false);
motor LeftRear = motor(PORT13, ratio6_1, true);

*/