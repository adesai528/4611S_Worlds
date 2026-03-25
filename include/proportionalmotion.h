void turnRightProportional(double target);
void turnLeftProportional(double target);
void driveForwardProportional(double distance);
void driveReverseProportional(double distance);
void driveForwardStraight(double distance, double speed);
void driveReverseStraight(double distance, double speed);
void turnRightToHeading(double targetHeading);
void turnLeftToHeading(double targetHeading);
void turnLeftToHeadingTurn(double targetHeading);
void turnRightToHeadingTurn(double targetHeading);
void driveForwardPD(double distance, double speed);
void driveForwardStraightPD(double distance, double speed);
void driveToPointPID(double targetx, double targety, double maxVolt, double desiredHeading, double POSITION_TOLERANCE, bool useTongue);
#pragma once