#include "vex.h"
#include "robotconfig.h"
#include "common.h"
#include "odometry.h"
using namespace vex;
//EXTERN Motor Groups
extern motor_group LeftMotorGroup;
extern motor_group RightMotorGroup;

//EXTERN Sensors
extern inertial inert;
extern brain Brain;

void turnRightProportional(double target) {   
    //Propotional Turn Right
    inert.resetRotation();
    while(inert.rotation(degrees) < target) {
        double proportion = target - inert.rotation(degrees); 
        double kp = .35;
        double min_speed = .25;
        double speed = proportion * kp + min_speed; //one way to break out of the loop

        LeftMotorGroup.spin(fwd, speed, pct);
        RightMotorGroup.spin(reverse, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}

void turnLeftProportional(double target) {   
    //Propotional Turn Left
    inert.resetRotation();
    while(inert.rotation(degrees) > -target) {
        double proportion = target + inert.rotation(degrees); 
        double kp = .35;
        double min_speed = .25;
        double speed = proportion * kp + min_speed; //one way to break out of the loop

        LeftMotorGroup.spin(reverse, speed, pct);
        RightMotorGroup.spin(fwd, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}

void driveToPointPID(double targetx, double targety, double maxVolt, double desiredHeading, double POSITION_TOLERANCE, bool useTongue){
  extern digital_out tongue_piston;
  double cX = getXposition();
  double cY = getYposition();

  double dX = targetx - cX;
  double dY = -1 * targety - cY; //sync with jerry io coords

  double distanceError = hypot(dX, dY);
  double distancePreviousError = distanceError;
  double startingError = distanceError;

  double targetHeading = 57.2958 * atan2(dY, dX) + desiredHeading;

  double currentHeading = inert.heading();
  double headingError = wrapAngle180(targetHeading-currentHeading);
  double previousHeadingError = headingError;

  double dP;
  double dD;

  double tP;
  double tD;

  double leftPower;
  double rightPower;

  double timeout = 0; //exit condition
  double pointDrivekP = 3;
  double pointDrivekD = 0.03;
  double pointTurnkP = 0.5;
  double pointTurnkD = 0.1;


  while (timeout <= (1000 + startingError * 50)) {
    cX = getXposition();
    cY = getYposition();

    dX = targetx - cX;
    dY = -1 * targety - cY;

    targetHeading = 57.2958*atan2(dY, dX) + desiredHeading; //from radians to degrees
    currentHeading = inert.heading();

    distanceError = hypot(dX, dY); 
    headingError =  wrapAngle180(targetHeading - currentHeading);

    dP = distanceError * pointDrivekP;
    dD = (distanceError - distancePreviousError) * pointDrivekD;

    tP = headingError * pointTurnkP;
    tD = (headingError - previousHeadingError) * pointTurnkD;

    if (distanceError <= POSITION_TOLERANCE){
      break;
    }
    if (distanceError <= 18 && useTongue){ //tongue blocks
      tongue_piston.set(true);
    }

    distancePreviousError = distanceError;
    previousHeadingError = headingError;

    if (fabs(desiredHeading - 180) < 0.001){
      leftPower = dP + dD + (tP + tD);
      rightPower = dP + dD - (tP + tD);
    } else{
      leftPower = dP + dD - (tP + tD);
      rightPower = dP + dD + (tP + tD);
    }
    
    leftPower = leftPower * 0.12;
    rightPower = rightPower * 0.12;

    if (leftPower > maxVolt){ leftPower = maxVolt;}
    if (leftPower < -maxVolt){ leftPower = -maxVolt;}
    
    if (rightPower > maxVolt){ rightPower = maxVolt;}
    if (rightPower < -maxVolt){ rightPower = -maxVolt;}

    if (desiredHeading == 180){
      leftPower = -leftPower;
      rightPower = -rightPower;
    }

    LeftMotorGroup.spin(forward, leftPower, volt);
    RightMotorGroup.spin(forward, rightPower, volt);

    timeout += 20;
    wait(20, msec);
  }
  LeftMotorGroup.stop(hold);
  RightMotorGroup.stop(hold);
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

void turnRightToHeadingTurn(double targetHeading){
    double kp = .5;
    targetHeading = wrapAngle(targetHeading);

    double currentHeading = wrapAngle(inert.heading(degrees));
    double error = counterclockwiseDistance(currentHeading, targetHeading);
    double speed = error * kp;

    while(fabs(error) > 2.0){
        currentHeading = wrapAngle(inert.heading(degrees));
        error = counterclockwiseDistance(currentHeading, targetHeading);
        speed = error * kp;
        LeftMotorGroup.spin(forward, speed, pct);
        RightMotorGroup.spin(reverse, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);   
}

void driveForwardProportional(double distance) {   //inches
    //Drive Forward Proportional
    double kp = .05;
    double min_speed = .25;
    LeftMotorGroup.resetPosition();
  
    //Convert Inches to Motor Encoder Degrees
    double target = inchesToDegrees(distance); 

    while(LeftMotorGroup.position(degrees) < target) {
        double proportion = target - LeftMotorGroup.position(degrees); 
        double speed = proportion * kp + min_speed; //one way to break out of the loop

        LeftMotorGroup.spin(fwd, speed, pct);
        RightMotorGroup.spin(fwd, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}

void driveReverseProportional(double distance) {   //inches
    //Drive Forward Proportional
    double kp = .2;
    double min_speed = .25;
    LeftMotorGroup.resetPosition();
  
    //Convert Inches to Motor Encoder Degrees
    double target = inchesToDegrees(distance); 

    while(LeftMotorGroup.position(degrees) > -target) {
        double proportion = target + LeftMotorGroup.position(degrees); 
        double speed = proportion * kp + min_speed; //one way to break out of the loop

        LeftMotorGroup.spin(reverse, speed, pct);
        RightMotorGroup.spin(reverse, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}

void driveForwardStraight(double distance, double speed) {    //inches
    inert.resetRotation();
    double targetRotation = inert.rotation(degrees); //save heading

    LeftMotorGroup.resetPosition();
    double targetDistance = inchesToDegrees(distance); 

    while(LeftMotorGroup.position(degrees) < targetDistance) {
        double error = targetRotation - inert.rotation(degrees);
        double kp = 1;
        double leftSpeed = speed + (error * kp);
        double rightSpeed = speed - (error * kp);

        LeftMotorGroup.spin(fwd, leftSpeed, pct);
        RightMotorGroup.spin(fwd, rightSpeed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}

void driveReverseStraight(double distance, double speed) {    //inches
    inert.resetRotation();
    double targetRotation = inert.rotation(degrees); //save heading

    LeftMotorGroup.resetPosition();
    double targetDistance = inchesToDegrees(distance); 

    while(LeftMotorGroup.position(degrees) > -targetDistance) {
        double error = targetRotation - inert.rotation(degrees);
        double kp = .2;

        double leftSpeed = speed - (error * kp);
        double rightSpeed = speed + (error * kp);

        LeftMotorGroup.spin(reverse, leftSpeed, pct);
        RightMotorGroup.spin(reverse, rightSpeed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}

void turnRightToHeading(double targetHeading){
    double kp = .42;
    targetHeading = wrapAngle(targetHeading);

    double currentHeading = wrapAngle(inert.heading(degrees));
    double error = clockwiseDistance(currentHeading, targetHeading);
    double speed = error * kp;

    while(fabs(error) > 2.0){
        currentHeading = wrapAngle(inert.heading(degrees));
        error = clockwiseDistance(currentHeading, targetHeading);
        speed = error * kp;
        LeftMotorGroup.spin(forward, speed, pct);
        RightMotorGroup.spin(reverse, speed, pct);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);   
}   

void turnLeftToHeading(double targetHeading){
    double kp = .42;
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

void driveForwardPD(double distance, double max_speed) {   //inches
    //Drive Forward Proportional
    double kp = .2;
    double kd = 2;
    double min_speed = .25;
    double speed = max_speed;

    //Convert Inches to Motor Encoder Degrees
    double target = inchesToDegrees(distance); 

    double derivative;
    double error = target - LeftMotorGroup.position(degrees);
    double previousError = error;

    LeftMotorGroup.resetPosition();

    while(fabs(error) > 2.0) { //
        previousError = error;
        error = target - LeftMotorGroup.position(degrees); 
        derivative = error - previousError;
        speed = error*kp + derivative*kd; 
        if (speed > max_speed) speed = max_speed;
        if (speed < min_speed) speed = min_speed;
        LeftMotorGroup.spin(fwd, speed, pct);
        RightMotorGroup.spin(fwd, speed, pct);
        wait(10, msec);
    }
    LeftMotorGroup.stop(coast);
    RightMotorGroup.stop(coast);
}

void driveForwardStraightPD(double distance, double max_speed) {   //inches
    //Drive Forward Proportional
    LeftMotorGroup.resetPosition();
    double target_d = inchesToDegrees(distance); //Convert Inches to Motor Encoder Degrees

    double kp_d = .5;
    double kd_d = .05;
    double min_speed = .25;
    double speed = max_speed;

    double derivative_d;
    double error_d = target_d - LeftMotorGroup.position(degrees);
    double previousError_d = error_d;

    inert.resetRotation();
    double target_h = inert.rotation(degrees);    
    double speed_correction = 0.0;

    double kp_h = .5;
    double kd_h = .05;

    double derivative_h;
    double error_h = target_h - inert.rotation(degrees);
    double previousError_h = error_h; 

    while(fabs(error_d) > 2.0) { //
        //distance pd calcs
        previousError_d = error_d;
        error_d = target_d - LeftMotorGroup.position(degrees); 
        speed = error_d*kp_d + derivative_d*kd_d; //one way to break out of the loop
        derivative_d = error_d - previousError_d;
        
        //correction pd calcs
        previousError_h = error_h;
        error_h = target_h - inert.rotation(degrees); 
        speed_correction = error_h*kp_h + derivative_h*kd_h;
        derivative_h = error_h - previousError_h;
    
        if (speed > max_speed) speed = max_speed;
        if (speed < min_speed) speed = min_speed;

        LeftMotorGroup.spin(fwd, speed + speed_correction, pct);
        RightMotorGroup.spin(fwd, speed - speed_correction, pct);
    
        wait(10, msec);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);
}

void turnRightToHeadingPD(double targetHeading){
    double kp = .3;
    double kd = .01;
    targetHeading = wrapAngle(targetHeading);

    double derivative, speed;
    double currentHeading = wrapAngle(inert.heading(degrees));
    double error = clockwiseDistance(currentHeading, targetHeading);
    double previousError = error; 

    while(fabs(error) > 2.0){
        currentHeading = wrapAngle(inert.heading(degrees));
        previousError = error;
        error = clockwiseDistance(currentHeading, targetHeading);
        derivative = error - previousError;
        speed = error * kp + derivative * kd;
        LeftMotorGroup.spin(forward, speed, pct);
        RightMotorGroup.spin(reverse, speed, pct);
        wait(10, msec);
    }
    LeftMotorGroup.stop(brake);
    RightMotorGroup.stop(brake);   
}