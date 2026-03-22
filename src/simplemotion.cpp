#include "vex.h"
#include "robotconfig.h"
using namespace vex;
//EXTERN Motor Groups
extern motor_group LeftMotorGroup;
extern motor_group RightMotorGroup;

//EXTERN Sensors
extern inertial inert;

void driveForwardSimple(double distance, double speed) { //Drive Forward Simple
   LeftMotorGroup.resetPosition();
   while(LeftMotorGroup.position(degrees)<distance){
        LeftMotorGroup.spin(fwd, speed, pct);
        RightMotorGroup.spin(fwd, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}

void driveReverseSimple(double distance, double speed) { //Drive Reverse Simple 
    LeftMotorGroup.resetPosition();
    while(LeftMotorGroup.position(degrees) > -distance){
         LeftMotorGroup.spin(reverse, speed, pct);
         RightMotorGroup.spin(reverse, speed, pct);
     }
     LeftMotorGroup.stop(brake);
     RightMotorGroup.stop(brake);
 }
 
void turnRightSimple(double target, double speed) { //Simple Turn Right
    inert.resetRotation();
    wait(.25, sec); //Sometimes Inertial/Gyro Sensors need some time to settle
    while(inert.rotation(degrees) < target) {
        LeftMotorGroup.spin(fwd, speed, pct);
        RightMotorGroup.spin(reverse, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}

void turnLeftSimple(double target, double speed) { //Simple Turn Left
    inert.resetRotation();
    wait(.25, sec); //Sometimes Inertial or Gyro Sensors need some time to settle
    while(inert.rotation(degrees) > -target) {
        LeftMotorGroup.spin(reverse, speed, pct);
        RightMotorGroup.spin(fwd, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}