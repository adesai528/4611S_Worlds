/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       arjundesai                                                */
/*    Created:      3/17/2026, 8:54:52 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robotconfig.h"
#include "common.h"
#include "simplemotion.h"
#include "proportionalmotion.h"
#include "odometry.h"
using namespace vex;
competition Competition;
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

void pre_auton(void) {
  tongue_piston.set(false);
  scoring_piston.set(true);
  left_descore.set(false);
  right_descore.set(false);
  inert.calibrate();
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "Selected Auton: %d");
    Brain.Screen.printAt(5, 40, "Auton 1: Left AWP");
    Brain.Screen.printAt(5, 60, "Auton 2: Solo Auton");
    Brain.Screen.printAt(5, 80, "Auton Skills: Dead Reckoning");
    Brain.Screen.printAt(5, 100, "Auton Skills: Inertial");
    task::sleep(50);
  }


void autonomous(void) {
  inert.setHeading(270, degrees);
  //Auto Here
}

void usercontrol(void) {
tongue_piston.set(false);
scoring_piston.set(true);
left_descore.set(false);
right_descore.set(false);
middle_descore.set(false);

while (1) {
LeftMotorGroup.spin(forward, (Controller1.Axis3.position() + ((Controller1.Axis1.position() * 0.75)))*0.12, volt);
RightMotorGroup.spin(forward, (Controller1.Axis3.position() - ((Controller1.Axis1.position() * 0.75)))*0.12, volt);
if (Controller1.ButtonX.pressing()) { //Score middle goal
IntakeFrontGroup.spin(forward, 100, pct);
Outtake.spin(reverse, 100, pct);
scoring_piston.set(false); //Retract scoring piston to score middle goal
} else if (Controller1.ButtonB.pressing()) { //Outtake all gems from bot
IntakeFrontGroup.spin(reverse, 100, pct);
Outtake.spin(forward, 100, pct);
scoring_piston.set(true); //Extend scoring piston to outtake from middle goal
} else if (Controller1.ButtonL1.pressing()) { //Score top goal
IntakeFrontGroup.spin(forward, 100, pct);
Outtake.spin(reverse, 100, pct);
scoring_piston.set(true); //Extend scoring piston to score top goal
} else if (Controller1.ButtonL2.pressing()) { //Outtake all gems from bot
IntakeFrontGroup.spin(reverse, 100, pct);
Outtake.spin(forward, 100, pct);
scoring_piston.set(true); //Extend scoring piston to outtake from top goal
} else if (Controller1.ButtonR1.pressing()) { //Storage (Intake gems to top but hold there)
IntakeFrontGroup.spin(forward, 100, pct);
scoring_piston.set(false); //Retract scoring piston to store gems
} else if (Controller1.ButtonR2.pressing()) { //Outtake all gems from bot
  IntakeFrontGroup.spin(reverse, 100, pct);
  Outtake.spin(forward, 100, pct);
  scoring_piston.set(false); //Retract scoring piston to store gems
} else {
  IntakeFrontGroup.stop(brake);
  Outtake.stop(brake);
}
}
}

void singlebutton() { //Toggle pistons with button presses
while(true) {
  if(Controller1.ButtonLeft.pressing()){
  while(Controller1.ButtonLeft.pressing()){
      task::sleep(10);
    }
    middle_descore_bool = !middle_descore_bool;
  }
  middle_descore.set(middle_descore_bool);

  if(Controller1.ButtonDown.pressing()){
  while(Controller1.ButtonDown.pressing()){
      task::sleep(10);
    }
    tongue_bool = !tongue_bool;
  }
  tongue_piston.set(tongue_bool);


  if(Controller1.ButtonUp.pressing()){
    waitUntil(!Controller1.ButtonUp.pressing());
    left_descore.set(!left_descore.value());
    right_descore.set(!right_descore.value());
  }

  task::sleep(20);
  }
  }

int main() {
  thread a(singlebutton);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}

/*
BUTTON OVERVIEW FOR DRIVERS:

UP: SINGLE BUTTON HIGH DESCORE
DOWN: SINGLE BUTTON TONGUE
LEFT: SINGLE BUTTON MIDDLE DESCORE
RIGHT: UNUSED
X: SCORE MIDDLE GOAL
B: OUTTAKE ALL GEMS FROM BOT
Y: UNUSED
A: UNUSED
L1: SCORE TOP GOAL
L2: OUTTAKE ALL GEMS FROM BOT
R1: STORAGE (INTAKE GEMS TO TOP BUT HOLD THERE)
R2: OUTTAKE ALL GEMS FROM BOT
CONTROLLER JOYSTICK 3: FORWARD AND BACKWARD
CONTROLLER JOYSTICK 1: LEFT AND RIGHT

*/
