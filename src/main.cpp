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
int current_auton_selection = 0;
bool auto_started = false;

//Pneumatics Pistons
digital_out tongue_piston = digital_out(Brain.ThreeWirePort.H); //Tongue
digital_out descore = digital_out(Brain.ThreeWirePort.F); //High Descore
digital_out scoring_piston = digital_out(Brain.ThreeWirePort.A); //Scoring
digital_out middle_descore = digital_out(Brain.ThreeWirePort.D); //Middle Descore

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
// motor RightFront = motor(PORT18, ratio6_1, true); //done
// motor RightMiddle = motor(PORT19, ratio6_1, true); //done
// motor RightRear = motor(PORT21, ratio6_1, false); //done
// motor LeftFront = motor(PORT11, ratio6_1, false); //done
// motor LeftMiddle = motor(PORT12, ratio6_1, false); //done
// motor LeftRear = motor(PORT13, ratio6_1, true); //done


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
  descore.set(false);
  inert.calibrate();
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(5, 20, "Selected Auton: %d");
  Brain.Screen.printAt(5, 40, "Auton 1: Left AWP");
  Brain.Screen.printAt(5, 60, "Auton 2: Solo Auton");
  Brain.Screen.printAt(5, 80, "Auton Skills: Dead Reckoning");
  Brain.Screen.printAt(5, 100, "Auton Skills: Inertial");
  task::sleep(20);
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
        Brain.Screen.printAt(5, 140, "One Inch");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

void turnLeftToHeadingTurn(double targetHeading){
    double kp = .5;
    targetHeading = wrapAngle(targetHeading);

    double currentHeading = wrapAngle(inert.heading(degrees));
    double error = counterclockwiseDistance(currentHeading, targetHeading);
    double speed = error * kp;

    while(fabs(error) > 2.0){
        currentHeading = wrapAngle(inert.heading(degrees));
        error = counterclockwiseDistance(currentHeading, targetHeading);
        speed = error * kp;
        LeftMotorGroup.spin(reverse, speed, pct);
        RightMotorGroup.spin(forward, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);   
}
void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection){ 
    case 0:
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
    case 1: 
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
    case 2:
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
    break;
  case 3:
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
  case 4:
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
  case 5:
    break;
  case 6:
    break;
  case 7:
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
      Outtake.spin(reverse, 1.5, volt);
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
      waitUntil(!Controller1.ButtonUp.pressing());
      high_descore_bool = !high_descore_bool;
      descore.set(high_descore_bool);
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
