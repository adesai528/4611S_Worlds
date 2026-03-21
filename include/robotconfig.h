#include "vex.h"

using namespace vex;

brain Brain;
controller Controller1 = controller(primary);
//X,Y are useless, B = tongue, x and b for mid score, left and right mid descore

//Bools
bool high_descore_bool = false;
bool middle_descore_bool = false;
bool tongue_bool = false;
//Pneumatics Pistons
digital_out tongue_piston = digital_out(Brain.ThreeWirePort.D); //Tongue
digital_out left_descore = digital_out(Brain.ThreeWirePort.F); //Left Descore
digital_out right_descore = digital_out(Brain.ThreeWirePort.G); //Right Descore
digital_out scoring_piston = digital_out(Brain.ThreeWirePort.B); //Scoring
digital_out middle_descore = digital_out(Brain.ThreeWirePort.E); //Middle Descore

//Sensors
inertial inert = inertial(PORT5);
distance distanceFront  = distance(PORT10);
distance distanceBack = distance(PORT2);
distance distanceLeft = distance(PORT4);
distance distanceRight = distance(PORT6);
distance distanceDown = distance(PORT20);


//Individual Motors
motor RightFront = motor(PORT18, ratio6_1, false); //done
motor RightMiddle = motor(PORT19, ratio6_1, true); //done
motor RightRear = motor(PORT21, ratio6_1, false); //done
motor LeftFront = motor(PORT13, ratio6_1, true); //done
motor LeftMiddle = motor(PORT12, ratio6_1, false); //done
motor LeftRear = motor(PORT11, ratio6_1, true); //done

//Intake Motors
motor FrontIntakeRight = motor(PORT17, ratio6_1, true); //done
motor FrontIntakeLeft = motor(PORT14, true); //done
motor Outtake = motor(PORT9, false); //done

//Motor Groups
motor_group LeftMotorGroup = motor_group(LeftFront, LeftMiddle, LeftRear);
motor_group RightMotorGroup = motor_group(RightFront, RightMiddle, RightRear);
motor_group IntakeFrontGroup = motor_group(FrontIntakeRight, FrontIntakeLeft);
motor_group AllMotorGroup = motor_group(LeftFront, RightFront, LeftMiddle, RightMiddle, LeftRear, RightRear);
motor_group TrulyAllMotorGroup = motor_group(LeftFront, RightFront, LeftMiddle, RightMiddle, LeftRear, RightRear, FrontIntakeRight, FrontIntakeLeft, Outtake);
