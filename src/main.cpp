#include "vex.h"
#include "robotconfig.h"
#include "common.h"
#include "simplemotion.h"
#include "proportionalmotion.h"
#include "odometry.h"
using namespace vex;
competition Competition;
controller Controller1 = controller(primary);

//DEFINED Variables
bool high_descore_bool = false;
bool middle_descore_bool = false;
bool tongue_bool = false;
bool auto_started = false;
int current_auton_selection = 0;
const char* autonNames[] = {"LEFTLM + Push", "RIGHTLB + Push", "Skills 20", "RIGHTCLUSTERL + Push", "LEFTLPUSH", "RIGHTLPUSH"};

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
    if(Controller1.ButtonLeft.pressing()) { //Controller Left Button For Middle Descore Toggle
      waitUntil(!Controller1.ButtonLeft.pressing());
      middle_descore_bool = !middle_descore_bool;
      middle_descore.set(middle_descore_bool);
    }

    if(Controller1.ButtonDown.pressing()) { //Controller Down Button For Tongue Toggle
      waitUntil(!Controller1.ButtonDown.pressing());
      tongue_bool = !tongue_bool;
      tongue_piston.set(tongue_bool);
    }

    //  if(Controller1.ButtonA.pressing()) { //Controller A Button For High Descore Toggle
    //   waitUntil(!Controller1.ButtonA.pressing());
    //   high_descore_bool = !high_descore_bool;
    //   descore.set(high_descore_bool);
    // }
    
    if(Controller1.ButtonA.pressing()) { //Controller A Button For High Descore Toggle
      descore.set(false);
      waitUntil(!Controller1.ButtonA.pressing());
      descore.set(true);
      task::sleep(200); // Debounce delay
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
    Brain.Screen.printAt(5, 20, "THIS CODE BETTER LOCK IN OR I'M GONNA BE MAD");
    Brain.Screen.printAt(5, 40, "Battery Percentage:" "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    Brain.Screen.printAt(5, 140, autonNames[current_auton_selection]);
    if(Brain.Screen.pressing() || Controller1.ButtonY.pressing()){
      while(Brain.Screen.pressing() || Controller1.ButtonY.pressing()) {}
      current_auton_selection ++;
      if(current_auton_selection == 6) current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

void autonomous(void) {
  auto_started = true;
  descore.set(true);
  scoring_piston.set(false);
  switch(current_auton_selection){ 
    case 0: //LEFTLM + Push
    initializeOdometry(-58, 6, 270);
    driveToPointPID(-58, 41.36314159265, 7, 0, 7, true, false);
    turnLeftToHeading(180);
    IntakeFrontGroup.spin(forward, 100, pct);
    AllMotorGroup.spin(forward, 20, pct);
    wait(1025, msec);
    AllMotorGroup.stop();
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    driveToPointPID(-15, 34.125, 10, 180, 18, true, false);
    AllMotorGroup.spin(reverse, 40, pct);
    scoring_piston.set(true);
    tongue_piston.set(false);
    wait(250, msec);
    AllMotorGroup.stop(brake);
    Outtake.spin(reverse, 100, pct);
    wait(750, msec);
    Outtake.spin(forward, 50, pct);
    AllMotorGroup.setVelocity(65, pct);
    AllMotorGroup.spinFor(forward, 500, deg);
    turnLeftToHeadingSlowestKP(70);
    wait(100, msec);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    wait(75, msec);
    driveToPointPID(-24, 7.25, 7, 0, 14, true, false);
    turnRightToHeadingSlowestKP(225);
    wait(100, msec);
    AllMotorGroup.spin(reverse, 60, pct);
    wait(550, msec);
    AllMotorGroup.stop(brake);
    scoring_piston.set(false);
    Outtake.spin(reverse, 100, pct);
    tongue_piston.set(false);
    wait(800, msec);
    Outtake.spin(forward, 50, pct);
    driveForwardPD(10, 30);
    middle_descore.set(true);
    driveReverseStraight(8, 32);
    if (inert.heading(degrees) < 225) {
      turnRightToHeadingSlowestKP(225);
    } else {
      turnLeftToHeadingSlowestKP(225);
    }
    // initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    // driveToPointPID(-48, 20, 5, 225, 20, false, false);
    // middle_descore.set(false);
    // turnLeftToHeadingSlowerKP(180);
    // descore.set(false);
    // driveReverseStraight(25, 50);
    middle_descore.set(false);
    driveForwardPD(25, 60);
    turnLeftToHeadingSlowestKP(180);
    wait(100, msec);
    LeftMotorGroup.spin(reverse, 30, pct);
    RightMotorGroup.spin(forward, 30, pct);
    wait(125, msec);
    AllMotorGroup.stop(hold);
    descore.set(false);
    wait(450, msec);
    AllMotorGroup.spin(reverse, 40, pct);
    wait(650, msec);
    AllMotorGroup.stop(hold);
    turnRightToHeading(180);
    AllMotorGroup.spin(reverse, 12.5, pct);
    wait(775, msec);
    AllMotorGroup.stop(hold);
    break;
  case 1: //RIGHTLB + Push
    initializeOdometry(-58, -6, 90);
    driveToPointPID(-58,-41.36314159265, 7, 0, 7, true, false);
    turnRightToHeadingSlowestKP(180);
    IntakeFrontGroup.spin(forward, 100, pct);
    AllMotorGroup.spin(forward, 25, pct);
    wait(1000, msec);
    AllMotorGroup.stop();
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    driveToPointPID(-18, -34.325, 11, 180, 18, true, false);
    AllMotorGroup.spin(reverse, 75, pct);
    scoring_piston.set(true);
    tongue_piston.set(false);
    wait(200, msec);
    AllMotorGroup.stop(brake);
    Outtake.spin(reverse, 100, pct);
    wait(750, msec);
    Outtake.spin(forward, 50, pct);
    AllMotorGroup.setVelocity(85, pct);
    AllMotorGroup.spinFor(forward, 400, deg);
    turnRightToHeadingSlowestKP(290);
    wait(100, msec);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    wait(75, msec);
    driveToPointPID(-24, -3.25, 7, 0, 14, true, false);
    turnRightToHeadingSlowestKP(315);
    wait(100, msec);
    tongue_piston.set(false);
    initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
    wait(75, msec);
    driveToPointPID(-3.5, -3.5, 5, 315, 14, false, false);
    middle_descore.set(true);
    IntakeFrontGroup.spin(reverse, 45, pct);
    if (inert.heading(degrees) < 315) {
      turnRightToHeadingSlowestKP(315);
      wait(75, msec);
      turnRightToHeadingSlowestKP(315);
    } else {
      turnLeftToHeadingSlowestKP(315);
      wait(75, msec);
      turnLeftToHeadingSlowestKP(315);
    }
  wait(2, sec);
  //IntakeFrontGroup.stop(brake);
    //INSERT PUSH CODE AFTER THIS! (ROBOT JUST FINISHED SCORING LOWER GOAL)
  // middle_descore.set(false);
  //   driveReverseSimple(30, 70);
  //   wait(50, msec);
  //   turnLeftToHeadingSlowestKP(180);
  //   wait(100, msec);
  //   LeftMotorGroup.spin(forward, 30, pct);
  //   RightMotorGroup.spin(reverse, 30, pct);
  //   wait(225, msec);
  //   AllMotorGroup.stop(hold);
  //   descore.set(false);
  //   wait(500, msec);
  //   AllMotorGroup.spin(reverse, 40, pct);
  //   wait(650, msec);
  //   AllMotorGroup.stop(hold);
  //   turnLeftToHeading(180);
  //   AllMotorGroup.spin(reverse, 12.5, pct);
  //   wait(775, msec);
  AllMotorGroup.stop(hold);   
  break;
  case 2: //Skills 20
  IntakeFrontGroup.spin(forward, 100, pct); 
  Outtake.spin(reverse, 100, pct);
  AllMotorGroup.spin(forward, 30, pct);
  wait(3, sec);
  AllMotorGroup.spin(reverse, 10, pct);
  wait(5, sec);
  AllMotorGroup.spin(forward, 20, pct);
  wait(3, sec);
  LeftMotorGroup.spin(forward, 15, pct);
  RightMotorGroup.spin(reverse, 15, pct);
  wait(775, msec);
  AllMotorGroup.stop(hold);
  
  break;
  case 3: //RIGHTCLUSTERL + Push
  initializeOdometry(-58, -6, 90);
  driveToPointPID(-24, -24, 7, 0, 7, true, false);
  turnRightToHeadingSlowerKP(110);
  initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
  driveToPointPID(-48, -48, 7, 250, 14, true, false);
  turnRightToHeadingSlowerKP(180);
  initializeOdometry(getXposition(), getYposition(), inert.heading(degrees));
  driveToPointPID(-18, -34.325, 11, 180, 18, true, false);
  break;
  case 4: //LEFTLPUSH
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
    //INSERT PUSH CODE AFTER THIS! (ROBOT JUST FINISHED SCORING FOUR BLOCKS)
  break;
  case 5: //RIGHTLPUSH
    initializeOdometry(-58, -6, 310);
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
    //INSERT PUSH CODE AFTER THIS! (ROBOT JUST FINISHED SCORING FOUR BLOCKS)
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
  thread d(initialize);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}

/* BUTTON OVERVIEW FOR DRIVERS:
  
  UP: UNUSED
  DOWN: TOGGLE BUTTON TONGUE
  LEFT: TOGGLE MIDDLE DESCORE
  RIGHT: UNUSED
  X: SCORE MIDDLE GOAL
  B: OUTTAKE ALL GEMS FROM BOT
  Y: UNUSED
  A: TOGGLE WINGS (HIGH DESCORE)
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