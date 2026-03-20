void updateOdometry();

float getXposition();

float getYposition();

void driveToPoint(float targetX, float targetY);

float getDistanceToTarget(float targetX, float targetY);

float getHeadingToTarget(float targetX, float targetY);

void driveDirectToPoint(float targetX, float targetY);

void driveToPointPID(float targetX, float targetY);

void initializeOdometry(float x, float y);

void initializeOdometry(float x, float y, float heading);
#pragma once