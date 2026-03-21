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
  task::sleep(20);
  }


void autonomous(void) {
  inert.setHeading(270, degrees);
  driveForwardPD(12, 100);
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
    if(Controller1.ButtonLeft.pressing()) {
      waitUntil(!Controller1.ButtonLeft.pressing());
      middle_descore_bool = !middle_descore_bool;
      middle_descore.set(middle_descore_bool);
    }

    if(Controller1.ButtonDown.pressing()) {
      waitUntil(!Controller1.ButtonDown.pressing());
      tongue_bool = !tongue_bool;
      tongue_piston.set(tongue_bool);
    }

    if(Controller1.ButtonUp.pressing()) {
      waitUntil(!Controller1.ButtonUp.pressing());
      high_descore_bool = !high_descore_bool;
      left_descore.set(high_descore_bool);
      right_descore.set(high_descore_bool);
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
