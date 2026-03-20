#include "vex.h"
#include "robotconfig.h"
using namespace vex;

// define used instances of motors and sensors as extern here because they are defined in robotconfig files
// Motor Groups
extern motor_group LeftMotorGroup;
extern motor_group RightMotorGroup;

//Sensors
extern inertial inert;

//Drive Forward Simple
void driveForwardSimple(double distance, double speed) {   
   LeftMotorGroup.resetPosition();
   while(LeftMotorGroup.position(degrees)<distance){
        LeftMotorGroup.spin(fwd, speed, pct);
        RightMotorGroup.spin(fwd, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}

//Drive Reverse Simple
void driveReverseSimple(double distance, double speed) {   
    LeftMotorGroup.resetPosition();
    while(LeftMotorGroup.position(degrees) > -distance){
         LeftMotorGroup.spin(reverse, speed, pct);
         RightMotorGroup.spin(reverse, speed, pct);
     }
     LeftMotorGroup.stop(brake);
     RightMotorGroup.stop(brake);
 }
 
//Simple Turn Right
void turnRightSimple(double target, double speed) {   
    inert.resetRotation();
    wait(.25, sec); //Sometimes Inertial/Gyro Sensors need some time to settle
    while(inert.rotation(degrees) < target) {
        LeftMotorGroup.spin(fwd, speed, pct);
        RightMotorGroup.spin(reverse, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}

//Simple Turn Left
void turnLeftSimple(double target, double speed) {   
    inert.resetRotation();
    wait(.25, sec); //Sometimes Inertial or Gyro Sensors need some time to settle
    while(inert.rotation(degrees) > -target) {
        LeftMotorGroup.spin(reverse, speed, pct);
        RightMotorGroup.spin(fwd, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}