#include "vex.h"
#include "robotconfig.h"
#include "common.h"
#include "simplemotion.h"
#include "proportionalmotion.h"
using namespace vex;
//EXTERN Motor Groups
extern motor_group LeftMotorGroup;
extern motor_group RightMotorGroup;

//EXTERN Sensors
extern inertial inert;
extern brain Brain;

//Globals
double XPosition = 0.0;
double YPosition = 0.0;
double heading = 0.0;
float lastEncoder = 0.0;

// Constants
const float WHEEL_DIAMETER = 2.75; // in inches
const float TICKS_PER_REV = 360.0;

void updateOdometry(){ //Run as a thread
    while(true) {
        // Get current heading in radians
        heading = inert.heading();
        float theta = heading * M_PI / 180.0;

        // Get encoder position
        float currentEncoder = (LeftMotorGroup.position(degrees) + RightMotorGroup.position(degrees)) /2;
        float deltaTicks = currentEncoder - lastEncoder;
        lastEncoder = currentEncoder;

        // Convert ticks to distance
        float distance = (deltaTicks / TICKS_PER_REV) * (M_PI * WHEEL_DIAMETER);

        // Update x, y using simple forward kinematics
        XPosition += distance * cos(theta);
        YPosition += distance * sin(theta);

        wait(50, msec);  // 20 Hz update rate
    }
}

float getXposition(){
    return XPosition;
}

float getYposition(){
    return YPosition;
}

float getDistanceToTarget(float targetX, float targetY) { // Returns the straight-line distance to the target point
    float deltaX = targetX - getXposition();
    float deltaY = targetY - getYposition();
    return sqrt(deltaX * deltaX + deltaY * deltaY);
  }
  
float getHeadingToTarget(float targetX, float targetY) { // Returns the heading you need to face the target point (0–360°)
    float deltaX = targetX - getXposition();
    float deltaY = targetY - getYposition();
  
    float angleToTargetDeg = atan2(deltaY, deltaX) * 180.0 / M_PI;
  
    if (angleToTargetDeg < 0) angleToTargetDeg += 360.0;
  
    return angleToTargetDeg;
}

void driveDirectToPoint(float targetX, float targetY){
    double dist = getDistanceToTarget(targetX, targetY);
    double degs = wrapAngle(getHeadingToTarget(targetX, targetY));

    if(degs>180){
        turnLeftToHeading(degs);
    }else{
        turnRightToHeading(degs);
    }
    driveForwardStraightPD(dist, 50);
}

void driveToPoint(float targetX, float targetY) {
  float KP_TURN = .50; // Proportional gain for turning
  float BASE_SPEED = 50; // Base driving speed
  float POSITION_TOLERANCE = 1.0; // Inches
    
  while (true) {
    float currentX = getXposition();  // Your odometry function
    float currentY = getYposition();
    float currentHeading = inert.heading();  // In degrees, 0-360

    float dx = targetX - currentX;
    float dy = targetY - currentY;
    float distance = sqrt(dx * dx + dy * dy);

    if (distance < POSITION_TOLERANCE){
        wait(150, msec);
      break; // Target reached
    }

    float angleToTarget = atan2(dy, dx) * 180.0 / M_PI; // Convert to degrees
    float angleError = wrapAngle180(angleToTarget - currentHeading);

    float turnAdjustment = angleError * KP_TURN;
    float leftSpeed = BASE_SPEED + turnAdjustment;
    float rightSpeed = BASE_SPEED - turnAdjustment;

    LeftMotorGroup.spin(fwd, leftSpeed, pct);
    RightMotorGroup.spin(fwd, rightSpeed, pct);

    wait(20, msec);
  }

  LeftMotorGroup.stop();
  RightMotorGroup.stop();
}

void driveToPointPID(float targetX, float targetY) {  //TODO: pos tol for motion chaining
    float kP_TURN = 0.1;
    float kI_TURN = 0.0;
    float kD_TURN = 0.1;
    
    float kP_DRIVE = 4;   // Stronger because distance is bigger numbers (inches)
    float kI_DRIVE = 0.03;   // Usually small or 0
    float kD_DRIVE = 0.6;   // Helps slow down as you approach
    
    float MAX_DRIVE_SPEED = 60; // Cap driving speed (percent)
    float MIN_DRIVE_SPEED = 10; // Don't go too slow  
    float POSITION_TOLERANCE = 1.0; // Inches

    float angleErrorSum = 0.0;
    float prevAngleError = 0.0;
    float distanceErrorSum = 0.0;
    float prevDistance = 0.0;
  
    while (true) {
      float currentX = getXposition();
      float currentY = getYposition();
      float currentHeading = inert.heading();  // 0 to 360 degrees
  
      float dx = targetX - currentX;
      float dy = targetY - currentY;
      float distance = sqrt(dx * dx + dy * dy);
  
      if (distance < POSITION_TOLERANCE) {
        wait(150, msec); //settling
        break;
      }
  
      // Angle PID
      float angleToTarget = atan2(dy, dx) * 180.0 / M_PI;
      float angleError = wrapAngle180(angleToTarget - currentHeading);
  
      angleErrorSum += angleError;
      float angleErrorRate = angleError - prevAngleError;
      prevAngleError = angleError;
  
      float turnAdjustment =
        (kP_TURN * angleError) +
        (kI_TURN * angleErrorSum) +
        (kD_TURN * angleErrorRate);
  
      // Distance PID
      distanceErrorSum += distance;
      float distanceErrorRate = distance - prevDistance;
      prevDistance = distance;
  
      float driveSpeed =
        (kP_DRIVE * distance) +
        (kI_DRIVE * distanceErrorSum) +
        (kD_DRIVE * distanceErrorRate);
  
      // Clip drive speed
      if (driveSpeed > MAX_DRIVE_SPEED) driveSpeed = MAX_DRIVE_SPEED;
      if (driveSpeed < MIN_DRIVE_SPEED) driveSpeed = MIN_DRIVE_SPEED;
  
      float leftSpeed = driveSpeed + turnAdjustment;
      float rightSpeed = driveSpeed - turnAdjustment;
  
      LeftMotorGroup.spin(fwd, leftSpeed, pct);
      RightMotorGroup.spin(fwd, rightSpeed, pct);
  
      wait(20, msec);
    }
  
    LeftMotorGroup.stop();
    RightMotorGroup.stop();
  }

  void initializeOdometry(float x, float y) {
    XPosition = x;
    YPosition = y;
  }

  void initializeOdometry(float x, float y, float heading) {
    XPosition = x;
    YPosition = y;
    inert.setHeading(heading, deg);
  }