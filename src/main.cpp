#include "vex.h"
#include "robotconfig.h"
#include "common.h"
#include "simplemotion.h"
#include "proportionalmotion.h"
#include "odometry.h"
using namespace vex;
competition Competition;
controller Controller1 = controller(primary);

//Bools and integers
bool high_descore_bool = false;
bool middle_descore_bool = false;
bool tongue_bool = false;
bool auto_started = false;
int current_auton_selection = 0;

//EXTERN Devices
extern brain Brain;
extern motor RightFront;
extern motor RightMiddle;
extern motor RightRear;
extern motor LeftFront;
extern motor LeftMiddle;
extern motor LeftRear;
extern motor FrontIntakeRight;
extern motor FrontIntakeLeft;
extern motor Outtake;
extern inertial inert;
extern distance distanceFront;
extern distance distanceBack;
extern distance distanceLeft;
extern distance distanceRight;
extern distance distanceDown;
extern digital_out tongue_piston;
extern digital_out descore;
extern digital_out scoring_piston;
extern digital_out middle_descore;
extern motor_group LeftMotorGroup;
extern motor_group RightMotorGroup;
extern motor_group IntakeFrontGroup;
extern motor_group AllMotorGroup;
extern motor_group TrulyAllMotorGroup;

void toggle() {
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

    if(Controller1.ButtonA.pressing()) {
      waitUntil(!Controller1.ButtonA.pressing());
      high_descore_bool = !high_descore_bool;
      descore.set(high_descore_bool);
    }

  task::sleep(20);
  }
}

void pre_auton(void) {
  tongue_piston.set(false);
  scoring_piston.set(false);
  descore.set(false);
  inert.calibrate();
  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "THIS CODE BETTER LOCK IN OR IM GONNA BE MAD");
    Brain.Screen.printAt(5, 40, "Battery Percentage:" "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 140, "LEFTLM");
        break;
      case 1:
        Brain.Screen.printAt(5, 140, "RIGHTLB");
        break;
      case 2:
        Brain.Screen.printAt(5, 140, "LEFTLMPush");
        break;
      case 3:
        Brain.Screen.printAt(5, 140, "RIGHT7LPUSH");
        break;
      case 4:
        Brain.Screen.printAt(5, 140, "LEFT4TOWER");
        break;
      case 5:
        Brain.Screen.printAt(5, 140, "SOLO LEFT AWP");
        break;
      case 6:
        Brain.Screen.printAt(5, 140, "SOLO RIGHT AWP");
        break;
      case 7:
        Brain.Screen.printAt(5, 140, "TESTING");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing() or Controller1.ButtonY.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

void autonomous(void) {
  auto_started = true;
  descore.set(true);
  scoring_piston.set(false);
  switch(current_auton_selection){ 
    case 0: //LEFTLM
    initializeOdometry(-58, 6, 270);
    driveToPointPID(-58, 41.36314159265, 7, 0, 7, true, false);
    turnLeftToHeading(180);
    IntakeFrontGroup.spin(forward, 100, pct);
    AllMotorGroup.spin(forward, 20, pct);
    wait(1.2, sec);
    AllMotorGroup.stop();
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    driveToPointPID(-19, 34.325, 10, 180, 18, true, false);
    AllMotorGroup.spin(reverse, 40, pct);
    scoring_piston.set(true);
    tongue_piston.set(false);
    wait(250, msec);
    AllMotorGroup.stop(brake);
    Outtake.spin(reverse, 100, pct);
    wait(750, msec);
    Outtake.spin(forward, 50, pct);
    AllMotorGroup.spinFor(forward, 125, deg);
    turnLeftToHeadingSlowerKP(70);
    wait(100, msec);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    wait(75, msec);
    driveToPointPID(-24, 7.25, 7, 0, 14, true, false);
    turnRightToHeadingSlowerKP(225);
    wait(100, msec);
    AllMotorGroup.spin(reverse, 60, pct);
    wait(550, msec);
    AllMotorGroup.stop(brake);
    scoring_piston.set(false);
    Outtake.spin(reverse, 100, pct);
    tongue_piston.set(false);
    wait(2500, msec);
    Outtake.spin(forward, 50, pct);
    driveForwardPD(10, 30);
    middle_descore.set(true);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    driveToPointPID(-3, 3, 10, 225, 18, true, false);
    break;
  case 1: //RIGHTLB
    initializeOdometry(-58, -6, 90);
    driveToPointPID(-58,-41.36314159265, 7, 0, 7, true, false);
    turnRightToHeading(180);
    IntakeFrontGroup.spin(forward, 100, pct);
    AllMotorGroup.spin(forward, 12, pct);
    wait(1.4, sec);
    AllMotorGroup.stop();
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    driveToPointPID(-18, -34.325, 11, 180, 18, true, false);
    AllMotorGroup.spin(reverse, 40, pct);
    scoring_piston.set(true);
    tongue_piston.set(false);
    wait(250, msec);
    AllMotorGroup.stop(brake);
    Outtake.spin(reverse, 100, pct);
    wait(750, msec);
    Outtake.spin(forward, 50, pct);
    AllMotorGroup.spinFor(forward, 400, deg);
    turnRightToHeadingSlowerKP(290);
    wait(100, msec);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    wait(75, msec);
    driveToPointPID(-24, -3.25, 7, 0, 14, true, false);
    turnRightToHeadingSlowerKP(315);
    wait(100, msec);
    tongue_piston.set(false);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    wait(75, msec);
    driveToPointPID(-3.5, -3.5, 5, 315, 14, false, false);
    IntakeFrontGroup.spin(reverse, 50, pct);
    if (inert.heading(degrees) < 315) {
      turnRightToHeadingSlowerKP(315);
      wait(75, msec);
      turnRightToHeadingSlowerKP(315);
    } else {
      turnLeftToHeadingSlowerKP(315);
      wait(75, msec);
      turnLeftToHeadingSlowerKP(315);
    }
    wait(2, sec);
    IntakeFrontGroup.stop(brake);
    break;
  case 2: //LEFTLMPush
    initializeOdometry(-58, 6, 270);
    driveToPointPID(-58, 40.9673612, 7, 0, 7, true, false);
    turnLeftToHeading(180);
    IntakeFrontGroup.spin(forward, 100, pct);
    AllMotorGroup.spin(forward, 15, pct);
    wait(1.1, sec);
    AllMotorGroup.stop();
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    driveToPointPID(-19, 34.325, 10, 180, 18, true, false);
    AllMotorGroup.spin(reverse, 40, pct);
    scoring_piston.set(true);
    tongue_piston.set(false);
    wait(250, msec);
    AllMotorGroup.stop(brake);
    Outtake.spin(reverse, 100, pct);
    wait(750, msec);
    Outtake.spin(forward, 50, pct);
    turnLeftToHeadingTurn(85);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    wait(75, msec);
    driveToPointPID(-24, 7.5, 7, 0, 14, true, false);
    turnRightToHeadingSlowerKP(225);
    AllMotorGroup.spin(reverse, 60, pct);
    wait(600, msec);
    AllMotorGroup.stop(brake);
    scoring_piston.set(false);
    Outtake.spin(reverse, 100, pct);
    tongue_piston.set(false);
    wait(1500, msec);
    Outtake.spin(forward, 50, pct);
    driveForwardPD(25, 60);
    turnLeftToHeadingSlowerKP(180);
    wait(100, msec);
    LeftMotorGroup.spin(reverse, 30, pct);
    RightMotorGroup.spin(forward, 30, pct);
    wait(250, msec);
    AllMotorGroup.stop(hold);
    descore.set(false);
    wait(500, msec);
    AllMotorGroup.spin(reverse, 45, pct);
    wait(650, msec);
    AllMotorGroup.stop(hold);
    turnRightToHeading(180);
    AllMotorGroup.spin(reverse, 12.5, pct);
    wait(775, msec);
    AllMotorGroup.stop(hold);
    break;
  case 3: //RIGHT7LPUSH
    inert.setHeading(90, degrees);
    driveForwardPD(9, 25);
    turnRightToHeading(0);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(forward, 100, pct);
    driveForwardPD(30, 25);
    turnRightToHeading(210);
    driveForwardStraight(35, 25);
    turnRightToHeading(180);
    AllMotorGroup.spin(forward, 35, pct);
    tongue_piston.set(true);
    wait(300, msec);
    AllMotorGroup.spin(forward, 20, pct);
    wait(500, msec);
    driveReverseStraight(28, 70);
    scoring_piston.set(true);
    Outtake.spin(reverse, 100, pct);
    wait(1500, msec);
    turnRightToHeadingTurn(80);
    driveForwardPD(4, 30);
    turnRightToHeading(0);
    descore.set(true);
    wait(500, msec);
    descore.set(false);
    driveForwardPD(20, 30);
    turnLeftToHeading(20);
    break;
  case 4: //LEFT4TOWER
    initializeOdometry(-58, 6, 270);
    driveToPointPID(-58, 42.75, 12, 0, 8, true, false);
    wait(50, msec);
    turnLeftToHeading(180);
    IntakeFrontGroup.spin(forward, 100, pct);
    scoring_piston.set(true);
    AllMotorGroup.spin(forward, 30, pct);
    wait(310, msec);
    AllMotorGroup.spin(forward, 20, pct);
    wait(660, msec);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    driveToPointPID(-19, 41.195, 15, 180, 9, true, false);
    AllMotorGroup.spin(reverse, 80, pct);
    tongue_piston.set(false);
    wait(250, msec);
    AllMotorGroup.stop(brake);
    Outtake.spin(reverse, 100, pct);
    wait(750, msec);
    Outtake.spin(forward, 50, pct);
    turnLeftToHeadingTurn(85);
    AllMotorGroup.spin(forward, 20, pct);
    wait(500, msec);
    AllMotorGroup.stop(brake);
    wait(75, msec);
    turnRightToHeadingSlowerKP(170);
    descore.set(false);
    AllMotorGroup.spin(reverse, 30, pct);
    wait(550, msec);
    AllMotorGroup.stop(brake);
    LeftMotorGroup.spin(forward, 12, pct);
    RightMotorGroup.spin(reverse, 12, pct);
    wait(500, msec);
    AllMotorGroup.stop(brake);
    driveReverseSimple(2, 15);
    LeftMotorGroup.spin(forward, 100, pct);
    RightMotorGroup.spin(reverse, 100, pct);
    Outtake.spin(forward, 50, pct);
    wait(100, msec);
    AllMotorGroup.stop(brake);
  break;
  case 5: //SOLO LEFT AWP
    inert.setHeading(270, degrees);
    initializeOdometry(-58, 6, 270);
    driveToPointPID(-58, 43.75, 12, 0, 8, true, false);
    wait(50, msec);
    turnLeftToHeading(180);
    IntakeFrontGroup.spin(forward, 100, pct);
    scoring_piston.set(true);
    AllMotorGroup.spin(forward, 30, pct);
    wait(310, msec);
    AllMotorGroup.spin(forward, 20, pct);
    wait(660, msec);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    driveToPointPID(-19, 41.195, 15, 180, 9, true, false);
    AllMotorGroup.spin(reverse, 80, pct);
    tongue_piston.set(false);
    wait(250, msec);
    AllMotorGroup.stop(brake);
    Outtake.spin(reverse, 100, pct);
    wait(800, msec);
    tongue_piston.set(false);
    Outtake.stop(brake);
    turnLeftToHeadingTurn(80);
    IntakeFrontGroup.spin(forward, 100, pct);
    driveForwardPD(26, 40);
    turnRightToHeading(225);
    driveReverseStraight(8, 35);
    scoring_piston.set(false);
    Outtake.spin(reverse, 70, pct);
    wait(700, msec);
    Outtake.stop(brake);
    driveForwardPD(35, 60);
    turnLeftToHeading(90);
    driveForwardPD(81, 100);
    turnRightToHeading(180);
    tongue_piston.set(true);
    Outtake.spin(forward, 100, pct);
    IntakeFrontGroup.spin(forward, 100, pct);
    AllMotorGroup.spin(forward, 50, pct);
    wait(400, msec);
    AllMotorGroup.spin(forward, 20, pct);
    wait(500, msec);
    driveReverseStraight(28, 70);
    scoring_piston.set(true);
    Outtake.spin(reverse, 100, pct);
    break;
  case 6: //SOLO RIGHT AWP
    initializeOdometry(-58, -6, 90);
    driveToPointPID(-58,-41.37314159265, 7, 0, 7, true, false);
    turnRightToHeading(180);
    IntakeFrontGroup.spin(forward, 100, pct);
    AllMotorGroup.spin(forward, 20, pct);
    wait(1, sec);
    AllMotorGroup.stop();
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    driveToPointPID(-19, -34.325, 10, 180, 18, true, false);
    AllMotorGroup.spin(reverse, 40, pct);
    scoring_piston.set(true);
    tongue_piston.set(false);
    wait(250, msec);
    AllMotorGroup.stop(brake);
    Outtake.spin(reverse, 100, pct);
    wait(750, msec);
    Outtake.spin(forward, 50, pct);
    AllMotorGroup.spinFor(forward, 125, deg);
    turnRightToHeadingSlowerKP(290);
    wait(100, msec);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    wait(75, msec);
    driveToPointPID(-24, -3.25, 7, 0, 14, true, false);
    turnRightToHeadingSlowerKP(315);
    wait(100, msec);
    tongue_piston.set(false);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    wait(75, msec);
    driveToPointPID(-3.25, -3.25, 5, 315, 14, false, false);
    if (inert.heading(degrees) < 315) {
      turnRightToHeadingSlowerKP(315);
      wait(75, msec);
      turnRightToHeadingSlowerKP(315);
    } else {
      turnLeftToHeadingSlowerKP(315);
      wait(75, msec);
      turnLeftToHeadingSlowerKP(315);
    }
    IntakeFrontGroup.spin(reverse, 50, pct);
    //place newer code above this
    wait(700, msec);
    Outtake.stop(brake);
    //outtake stop after scoring bottom goal
    driveReverseStraight(35, 60);
    turnRightToHeading(90);
    driveForwardPD(81, 100);
    turnLeftToHeading(180);
    tongue_piston.set(true);
    Outtake.spin(forward, 100, pct);
    IntakeFrontGroup.spin(forward, 100, pct);
    AllMotorGroup.spin(forward, 50, pct);
    wait(400, msec);
    AllMotorGroup.spin(forward, 20, pct);
    wait(500, msec);
    driveReverseStraight(28, 70);
    scoring_piston.set(true);
    Outtake.spin(reverse, 100, pct); 
    break;
  case 7: //TESTING
    initializeOdometry(-58, -6, 90);
    driveToPointPID(-58,-41.314159265, 7, 0, 7, true, false);
    turnRightToHeading(180);
    IntakeFrontGroup.spin(forward, 100, pct);
    AllMotorGroup.spin(forward, 20, pct);
    wait(1, sec);
    AllMotorGroup.stop();
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    driveToPointPID(-19, -34.325, 10, 180, 18, true, false);
    AllMotorGroup.spin(reverse, 40, pct);
    scoring_piston.set(true);
    tongue_piston.set(false);
    wait(250, msec);
    AllMotorGroup.stop(brake);
    Outtake.spin(reverse, 100, pct);
    wait(750, msec);
    Outtake.spin(forward, 50, pct);
    AllMotorGroup.spinFor(forward, 150, deg);
    turnRightToHeading(270);
    wait(100, msec);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    wait(75, msec);
    driveToPointPID(-24, -6.25, 7, 0, 14, true, false);
    turnRightToHeadingSlowerKP(315);
    wait(100, msec);
    tongue_piston.set(false);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    wait(75, msec);
    driveToPointPID(-4, -4, 5, 315, 14, false, false);
    IntakeFrontGroup.spin(reverse, 45, pct);
    break;
}

}

void usercontrol(void) {
  tongue_piston.set(false);
  scoring_piston.set(true);
  descore.set(false);
  middle_descore.set(false);
  LeftMotorGroup.setStopping(coast);
  RightMotorGroup.setStopping(coast);
  while (1) {
    LeftMotorGroup.spin(forward, (Controller1.Axis3.position() + ((Controller1.Axis1.position() * 0.75)))*0.12, volt);
    RightMotorGroup.spin(forward, (Controller1.Axis3.position() - ((Controller1.Axis1.position() * 0.75)))*0.12, volt);
    if (Controller1.ButtonB.pressing()) { //Score middle goal
      IntakeFrontGroup.spin(forward, 100, pct);
      Outtake.spin(reverse, 100, pct);
      scoring_piston.set(false); //Retract scoring piston to score middle goal
    } else if (Controller1.ButtonX.pressing()) { //Outtake all gems from bot
      IntakeFrontGroup.spin(reverse, 100, pct);
      Outtake.spin(forward, 100, pct);
      scoring_piston.set(true); //Extend scoring piston to outtake from middle goal
    } else if (Controller1.ButtonL1.pressing()) { //Score top goal
      IntakeFrontGroup.spin(forward, 100, pct);
      Outtake.spin(reverse, 100, pct);
      scoring_piston.set(true); //Extend scoring piston to score top goal
    } else if (Controller1.ButtonL2.pressing()) { //Outtake all gems from bot
      Outtake.spin(forward, 1.5, volt);
      Outtake.spin(forward, 100, pct);
      scoring_piston.set(true); //Extend scoring piston to outtake from top goal
    } else if (Controller1.ButtonR1.pressing()) { //Storage (Intake gems to top but hold there)
      Outtake.spin(forward, 1.5, volt);
      IntakeFrontGroup.spin(forward, 100, pct);
      scoring_piston.set(true); //Extend scoring piston to outtake from middle goal
    } else if (Controller1.ButtonR2.pressing()) { //Outtake all gems from bot
      IntakeFrontGroup.spin(reverse, 100, pct);
      Outtake.spin(forward, 100, pct);
    } else {
      IntakeFrontGroup.stop(coast);
      Outtake.stop(coast);
    }
  }
}

int main() {
  thread a(toggle);
  thread b(updateOdometry);
  thread c(controllerDisplay);
  inert.calibrate();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}

/* BUTTON OVERVIEW FOR DRIVERS:
  
  UP: UNUSED
  DOWN: SINGLE BUTTON TONGUE
  LEFT: SINGLE BUTTON MIDDLE DESCORE
  RIGHT: UNUSED
  X: SCORE MIDDLE GOAL
  B: OUTTAKE ALL GEMS FROM BOT
  Y: UNUSED
  A: WINGS
  L1: SCORE TOP GOAL
  L2: OUTTAKE ALL GEMS FROM BOT
  R1: STORAGE (INTAKE GEMS TO TOP BUT HOLD THERE)
  R2: OUTTAKE ALL GEMS FROM BOT
  CONTROLLER JOYSTICK 3: FORWARD AND BACKWARD
  CONTROLLER JOYSTICK 1: LEFT AND RIGHT

*/

/* STRATEGY FOR STRATEGISTS: 
  
  SOLO AUTON LEFT: Start left, go matchloader, score long goal, collect 3 gems middle, score middle, go right matchloader, score long goal.
  LEFTML: Start left, collect 3 gems middle, score middle, go matchloader, score long goal.
  LEFTLM: Start left, go matchloader, score long goal, collect 3 gems middle, score middle.
  RIGHTBL: Start right, collect 3 gems middle, score bottom.
  RIGHTLB: Start right, go matchloader, score long goal, collect 3 gems middle, score bottom.
  LEFT7LPUSH: (Start ML or LM), but use wings to descore against opponents.
  RIGHT7LPUSH: (Start BL or LB, but use wings to descore against opponents.
  SOLO RIGHT AWP: Start right, go matchloader, score long goal, collect 3 gems middle, score bottom, go left matchloader, score long goal.
  ONE INCH: Drive forward for an inch (for testing purposes only)

*/