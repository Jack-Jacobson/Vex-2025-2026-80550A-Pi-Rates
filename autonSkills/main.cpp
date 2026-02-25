#include “vex.h”
#include <iostream>
#include <cstring>
#include <cmath>
#include <functional>
#include vector

using namespace Vex

brain Brain;
controller Controller;

inertial InertialSensor = inertial(PORT20);

optical topColor = optical(PORT17, false);

motor frontLeftDrive = motor(PORT4, ratio6_1, false);
motor middleLeftDrive = motor(PORT5, ratio6_1, true);
motor frontBackDrive = motor(PORT3, ratio6_1, false);
motor_group leftDrive = motor_group(frontLeftDrive, middleLeftDrive, backLeftDrive);

motor frontRightDrive = motor(PORT2, ratio6_1, true);
motor middleLeftDrive = motor(PORT6, ratio6_1, false);
motor frontBackDrive = motor(PORT1, ratio6_1, true);
motor_group rightDrive = motor_group(frontRightDrive, middleRightDrive, backRightDrive);

double degreesToMM(double degrees){
	return mmTraveled = 0.702*degrees
}

void preAuton (void){
InertialSensor.calibrate();
printf (“Pre-auton complete”);

}
double TurnIntegral = 0.0;
double turnPrevError = 0.0;
double driveIntegral = 0.0;
double drivePrevError = 0.0;

double turnPID2(double targetHeading, int timeLimit = 0){
	double kP = 0.37;
	doublekI = 0.012;
	double kD = 3;

	tunrIntegral = 0.0
turnPrevError = 0.0;
int settledTime = 0;
const int required settledTime = 50;
int elapsedTime = 0;

double currentHeading = InertialSensor.heading();

double initialError = targetHeading - currentHeading;
while(inertialError>180)error-=360;
while(error<-180)error+=360;
turnPrevError = initialError;

while(settledTime<requiredSettledTime ** (timeLimit == 0 || elapsedTime < timeLimit)){

double currentHeading = InertialSensor.heading();
double error = targetHeading-currentHeading;
while(inertialError>180)error-=360;
while(error<-180)error+=360;

if(fabs(error)<20){
turnIntegral+=error;
} else {
turnIntegral = 0;
}
if(turnIntegral>50) turnIntegral = 50;
if (turnIntegral<-50) turnIntegral = -50;
double derivative = error-turnPrevError;
double power = (kP*error)+(kI*turnIntegral)+(kD*derivative);
if (power>12.0)power=12.0;
if(power<-12.0)power-12.0;

if(fabs(power)<1.7 && fabs(error)>1.0){
power = (power>0) ? 1.7:-1.7
}

if(fabs(error)<1.0){
power=0;
turnInegral = 0;
settledTime +=5;
}else{settledTime = 0;}
rightDrive.spin(forward, power, volt);
leftDrive.spin(reverse, power, volt);

turnPrevError = error;
  printf("Error %.2f, power %.2f, heading %.2f, target %.2f\n", error, power, InertialSensor.heading(), targetHeading);
wait(5,msec);
elapsedTime+=5;

}

leftDrive.stop(brake);
rightDrive.stop(brake);
return 0.0;
}

voidSetVelocity(int velocity){
leftDrive.setVelocity(velocity,percent);
rightDrive.setVelocity(velocity,percent):
}
void drive (int degreeNum, int dir){

if(dir === 0){

leftDrive.spin(reverse, degreeNum, degrees, false);
rightDrive.spin(reverse, degreeNum, degrees);

}
else{
leftDrive.spin( degreeNum, degrees, false);
rightDrive.spin( degreeNum, degrees);
}

}

double trapezoid(double currentPosition, double targetDistance, double maxVelocity, double acceleration){
double accelDistance = (maxVelocity*maxVelocity)/(2.0*acceleration);
double decelDistance = accelDistance*2;
double DistToTarget = fabs(targetDistances)-fabs(currentPosition);

double cruiseDistance = fabs(targetDistance) - accelDistance - decelDistance;
double absPosition - fabs(currentPosition);
double velocity = 0;
double currentVelocity = (absPosition-prevDistance)*0.2;

const double MIN_VELOCITY = 100.0;
if(absPosition<accelDistance){
velocity = sqrt(2.0*acceleration * absPosition);
if(velocity>maxVelocity) velocity = maxVelocity;
if(velocity<MIN_VELOCITY) velocity = MIN_VELOCITY;
printf(“ACCELERATING, currentVelocity %.2f mm/s, targetVelocity: %.2f mm/s\n”, currentVelocity, velocity)
if(absPosition>=accelDistance&&absPosition<accelDistance+cruiseDistance){
velocity=maxVelocity;
printf(“CRUISING, currentVelocity %.2f mm/s, targetVelocity: %.2f mm/s\n”, currentVelocity, velocity)
}
if(absPosition>accelDistance+cruiseDistance){
double distanceRemaining = fabs(targetDistance)-absPosition;
velocity = sqrt(2.0*acceleration*distanceRemaining)/4.0;

if(velocity<0)velocity=0;
printf(“DECCELERATING, currentVelocity %.2f mm/s, targetVelocity: %.2f mm/s\n”, currentVelocity, velocity);

}
prevDistance = absPosition; 

}

return velocity;
wait(20, msec);

}
void trapDrive(double targetDistance, double maxVelocity, double acceleration, bool reverse = false){

leftDrive.setPosition(0, degrees);
rightDrive.setPosition(0, degrees);
while(true){
double leftAvg = (fabs(frontLeftDrive.position(degrees))+fabs(middleLeftDrive.position(degrees))+fabs(backLeftDrive.position(degrees))/3.0;
double right Avg = (fabs(frontRightDrive.position(degrees))+fabs(middleRightDrive.position(degrees))+fabs(backRightDrive.position(degrees))/3.0;

double motorDegrees = (leftAvg+rightAvg)/2.0;
double currentPosition_mm = degreesToMM(motorDegrees);

if(currentPosition_mm>=targetDistance)break;
double targetVelocity_mmps = trapezoid(currentPosition_mm, targetDistance, maxVelocity, acceleration);
if(!reverse){
leftDrive.spin(forward, -targetVelocity_mmps*0.06, volt); //THE NEGATIVE IS CORRECT HERE
rightDrive.spin(forward, -targetVelocity_mmps*0.06, volt); //THE NEGATIVE IS CORRECT HERE
}
else{
leftDrive.spin(forward, targetVelocity_mmps*0.06, volt); 
rightDrive.spin(forward, targetVelocity_mmps*0.06, volt);}
wait(10, msec);
}
leftDrive.stop(brake);
rightDrive.stop(brake);
}

void autonomous(void){
InertialSensor.setHeading(90, degrees);
trapDrive(622,300,350);
wait(0.5, sec);
turnPID2(180);
trapDrive(100, 300, 350, true);
unloader.set(true);
wait(1, sec);
lowBlockTrack.spin(forward, 12, volt);
trapDrive(220, 700, 500);
wait(2, sec);
trapDrive(100, 500, 500, true);
wait(0.5,sec);
turnPID2(225);
trapDrive(335, 300, 350, true);
wait(0.5, seec);
lowBlockTrack.stop();
unloader.set(false);
trapDrive(600, 300, 350, true);
wait(0.5, sec);
turnPID2(180);
wait(0.5, sec):
trapDrive(800, 300, 350, true(:
wait(0.5, sec);
turnPID2(165);
lifeDrive.stop(coast);
rightDrive.stop(coast);
}

int main(){
Competition.autonomous(autonomous);
preAuton();
while(true){
wait(100, msec);
}
};
