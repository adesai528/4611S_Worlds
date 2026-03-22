#include "vex.h"
#include "robotconfig.h"
#include "common.h"
#include "simplemotion.h"
#include "proportionalmotion.h"
#include "odometry.h"
using namespace vex;

//Competition Global Instances
competition Competition;
brain Brain;
controller Controller1 = controller(primary);

//Bools and integers
bool high_descore_bool = false;
bool middle_descore_bool = false;
bool tongue_bool = false;
bool auto_started = false;
int current_auton_selection = 0;

//Pneumatics Pistons (Define in robotconfig.cpp later) - use extern and remove the definition in main.cpp
digital_out tongue_piston = digital_out(Brain.ThreeWirePort.H); //Tongue
digital_out descore = digital_out(Brain.ThreeWirePort.F); //High Descore
digital_out scoring_piston = digital_out(Brain.ThreeWirePort.A); //Scoring
digital_out middle_descore = digital_out(Brain.ThreeWirePort.D); //Middle Descore
distance distanceFront  = distance(PORT10);
distance distanceBack = distance(PORT2);
distance distanceLeft = distance(PORT4);
distance distanceRight = distance(PORT6);
distance distanceDown = distance(PORT20);

//External Devices
extern inertial inert;
extern motor RightFront;
extern motor RightMiddle;
extern motor RightRear;
extern motor LeftFront;
extern motor LeftMiddle;
extern motor LeftRear;
extern motor FrontIntakeRight;
extern motor FrontIntakeLeft;
extern motor Outtake;
extern motor_group LeftMotorGroup;
extern motor_group RightMotorGroup;
extern motor_group IntakeFrontGroup;
extern motor_group AllMotorGroup;
extern motor_group TrulyAllMotorGroup;

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
  scoring_piston.set(true);
  descore.set(false);
  inert.calibrate();
  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    Brain.Screen.printAt(5, 40, "Battery Percentage:" "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 140, "Solo Auton Left");
        break;
      case 1:
        Brain.Screen.printAt(5, 140, "LeftML");
        break;
      case 2:
        Brain.Screen.printAt(5, 140, "LeftLM");
        break;
      case 3:
        Brain.Screen.printAt(5, 140, "RightBL");
        break;
      case 4:
        Brain.Screen.printAt(5, 140, "RightLB");
        break;
      case 5:
        Brain.Screen.printAt(5, 140, "Left7LPush");
        break;
      case 6:
        Brain.Screen.printAt(5, 140, "Right7RPush");
        break;
      case 7:
      Brain.Screen.printAt(5, 140, "Solo Right AWP");
        break;
      case 8:
        Brain.Screen.printAt(5, 140, "One Inch");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing() or Controller1.ButtonRight.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 9){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection){ 
    case 0: //Solo Auton Left
    inert.setHeading(270, degrees);
    driveForwardPD(34, 70);
    tongue_piston.set(true);
    wait(100, msec);
    turnLeftToHeading(180);
    AllMotorGroup.spin(forward, 30, pct);
    wait(300, msec);
    AllMotorGroup.spin(forward, 10, pct);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(forward, 100, pct);
    wait(525, msec);
    driveReverseStraight(28.5, 70);
    wait(100, msec);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(reverse, 100, pct);
    wait(800, msec);
    tongue_piston.set(false);
    Outtake.stop(brake);
    turnLeftToHeadingTurn(80);
    IntakeFrontGroup.spin(forward, 100, pct);
    driveForwardPD(28, 30);
    turnRightToHeading(210);
    driveReverseStraight(9, 35);
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
    case 1: //LeftML
    inert.setHeading(270, degrees);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(forward, 100, pct);
    driveForwardPD(9, 25);
    turnRightToHeading(0);
    driveForwardPD(30, 25);
    turnLeftToHeading(210);
    driveReverseStraight(7, 25);
    scoring_piston.set(false);
    Outtake.spin(reverse, 70, pct);
    wait(2000, msec);
    driveForwardPD(40, 25);
    turnLeftToHeadingTurn(180);
    AllMotorGroup.spin(forward, 25, pct);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(forward, 100, pct);
    wait(1500, msec);
    driveReverseStraight(28, 60);
    scoring_piston.set(true);
    Outtake.spin(reverse, 100, pct); 
  break;
    case 2: //LeftLM
    inert.setHeading(270, degrees);
    driveForwardPD(34, 70);
    tongue_piston.set(true);
    wait(100, msec);
    turnLeftToHeading(180);
    AllMotorGroup.spin(forward, 30, pct);
    wait(300, msec);
    AllMotorGroup.spin(forward, 10, pct);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(forward, 100, pct);
    wait(525, msec);
    driveReverseStraight(28.5, 70);
    wait(100, msec);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(reverse, 100, pct);
    wait(800, msec);
    tongue_piston.set(false);
    Outtake.stop(brake);
    turnLeftToHeadingTurn(80);
    IntakeFrontGroup.spin(forward, 100, pct);
    driveForwardPD(28, 30);
    turnRightToHeading(210);
    driveReverseStraight(9, 35);
    scoring_piston.set(false);
    Outtake.spin(reverse, 70, pct);
    wait(2000, msec);
    driveForwardPD(10, 25);
    middle_descore.set(true);
    wait(500, msec);
    driveReverseStraight(12, 25);
    break;
  case 3: //RightBL
    inert.setHeading(90, degrees);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(forward, 100, pct);
    driveForwardPD(9, 25);
    turnRightToHeading(0);
    driveForwardPD(30, 25);
    turnRightToHeading(45);
    driveForwardStraight(7, 15);
    Outtake.spin(forward, 100, pct);
    IntakeFrontGroup.spin(reverse, 50, pct);
    wait(2000, msec);
    driveReverseStraight(40, 25);
    turnLeftToHeadingTurn(180);
    AllMotorGroup.spin(forward, 25, pct);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(forward, 100, pct);
    wait(1500, msec);
    driveReverseStraight(28, 60);
    scoring_piston.set(true);
    Outtake.spin(reverse, 100, pct);
    break;
  case 4: //RightLB
    inert.setHeading(90, degrees);
    driveForwardPD(34, 70);
    tongue_piston.set(true);
    wait(100, msec);
    turnLeftToHeading(180);
    AllMotorGroup.spin(forward, 30, pct);
    wait(300, msec);
    AllMotorGroup.spin(forward, 10, pct);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(forward, 100, pct);
    wait(525, msec);
    driveReverseStraight(28.5, 60);
    wait(100, msec);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(reverse, 100, pct);
    wait(800, msec);
    tongue_piston.set(false);
    Outtake.stop(brake);
    turnLeftToHeadingTurn(80);
    IntakeFrontGroup.spin(forward, 100, pct);
    driveForwardPD(28, 30);
    turnRightToHeading(45);
    driveForwardStraight(5, 15);
    Outtake.spin(forward, 70, pct);
    IntakeFrontGroup.spin(reverse, 40, pct);
    break;
  case 5: //Left7LPush - Arjun worked on this one, but it is untested and may not be fully functional. Use with caution.
    initializeOdometry(0, 0, 270);
    driveToPointPID(39, 0);
    break;
    turnRightToHeading(270);
    // IntakeFrontGroup.spin(forward, 100, pct);
    // Outtake.spin(forward, 100, pct);
    // driveForwardPD(30, 25);
    // turnLeftToHeading(210);
    // driveForwardStraight(35, 25);
    // turnLeftToHeading(180);
    // AllMotorGroup.spin(forward, 35, pct);
    // tongue_piston.set(true);
    // wait(300, msec);
    // AllMotorGroup.spin(forward, 20, pct);
    // wait(500, msec);
    // driveReverseStraight(28, 70);
    // scoring_piston.set(true);
    // Outtake.spin(reverse, 100, pct);
    // wait(1500, msec);
    // turnLeftToHeadingTurn(80);
    // driveForwardPD(4, 30);
    // turnLeftToHeading(0);
    // descore.set(true);
    // wait(500, msec);
    // descore.set(false);
    // driveForwardPD(20, 30);
    // turnRightToHeading(20);
    break;
  case 6: //Right7LPush
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
  case 7: //Solo Right AWP
    inert.setHeading(90, degrees);
    driveForwardPD(34, 70);
    tongue_piston.set(true);
    wait(100, msec);
    turnRightToHeading(180);
    AllMotorGroup.spin(forward, 30, pct);
    wait(300, msec);
    AllMotorGroup.spin(forward, 10, pct);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(forward, 100, pct);
    wait(525, msec);
    driveReverseStraight(28.5, 70);
    wait(100, msec);
    IntakeFrontGroup.spin(forward, 100, pct);
    Outtake.spin(reverse, 100, pct);
    wait(800, msec);
    tongue_piston.set(false);
    Outtake.stop(brake);
    turnRightToHeadingTurn(80);
    IntakeFrontGroup.spin(forward, 100, pct);
    driveForwardPD(28, 30);
    turnLeftToHeading(30);
    driveForwardStraight(4, 35);
    Outtake.spin(forward, 70, pct);
    IntakeFrontGroup.spin(reverse, 50, pct);
    wait(700, msec);
    Outtake.stop(brake);
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
  case 8:
    driveForwardPD(1, 25);
    break;
 }

}

void usercontrol(void) {
  tongue_piston.set(false);
  scoring_piston.set(true);
  descore.set(false);
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
      Outtake.spin(forward, 1.5, volt);
      Outtake.spin(forward, 100, pct);
      scoring_piston.set(true); //Extend scoring piston to outtake from top goal
    } else if (Controller1.ButtonR1.pressing()) { //Storage (Intake gems to top but hold there)
      Outtake.spin(forward, 1.5, volt);
      IntakeFrontGroup.spin(forward, 100, pct);
    } else if (Controller1.ButtonR2.pressing()) { //Outtake all gems from bot
      IntakeFrontGroup.spin(reverse, 100, pct);
      Outtake.spin(forward, 100, pct);
    } else {
      IntakeFrontGroup.stop(brake);
      Outtake.stop(brake);
    }
  }
}

int main() {
  thread a(singlebutton);
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