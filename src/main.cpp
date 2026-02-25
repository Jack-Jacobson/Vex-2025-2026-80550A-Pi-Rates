/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jokin                                                     */
/*    Created:      1/8/2026, 6:31:51 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <iostream>
#include <cstring>
#include <cmath>
#include <functional>
#include <vector>


using namespace vex;

competition Competition;

brain Brain;
controller Controller;

inertial InertialSensor = inertial(PORT20);

optical topColor = optical(PORT17, false);

motor frontLeftDrive = motor(PORT4, ratio6_1, false);
motor middleLeftDrive = motor(PORT5, ratio6_1, true);
motor backLeftDrive = motor(PORT3, ratio6_1, false);
motor_group leftDrive = motor_group(frontLeftDrive, middleLeftDrive, backLeftDrive);

motor frontRightDrive = motor(PORT2, ratio6_1, true);
motor middleRightDrive = motor(PORT6, ratio6_1, false);
motor backRightDrive = motor(PORT1, ratio6_1, true);
motor_group rightDrive = motor_group(frontRightDrive, middleRightDrive, backRightDrive);

motor lowBlockTrack = motor(PORT14, ratio6_1, true);
motor highBlockTrack = motor(PORT15, ratio6_1, true);

//Radio in 21

digital_out unloader = digital_out(Brain.ThreeWirePort.A);
digital_out descore = digital_out(Brain.ThreeWirePort.H);

// define your global instances of motors and other devices here

// Motion control constants
const double WHEEL_DIAMETER_MM = 1;  // actual 69.85 .75 inch wheels
const double GEAR_RATIO = 1;         
                                        // Based on actual test: 550mm moved, 229mm reported
const double MAX_MOTOR_RPM = 600;        // Blue cartridge max RPM
const double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * 1;  //replaced pi with 1 Distance per wheel rotation

 double degreesToMM(double degrees) {
  // printf("Degrees %.2f\n", degrees);
  // double mmTraveled = (degrees / 360.0) * WHEEL_CIRCUMFERENCE_MM;
    double mmTraveled = 0.702 * degrees; 

  return mmTraveled;
}

 double mmPerSecToRPM(double mmPerSec) {
  double wheelRPM = (mmPerSec / WHEEL_CIRCUMFERENCE_MM) * 60.0;
  return wheelRPM * GEAR_RATIO;
}

 double mmPerSecToPercent(double mmPerSec) {
  double motorRPM = mmPerSecToRPM(mmPerSec);
  return (motorRPM / MAX_MOTOR_RPM) * 100.0;
}

 double percentToMMPerSec(double percent) {
  double motorRPM = (percent / 100.0) * MAX_MOTOR_RPM;
  double wheelRPM = motorRPM / GEAR_RATIO;
  return (wheelRPM / 60.0) * WHEEL_CIRCUMFERENCE_MM;
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
 printf("Pre-autonomous setup complete.\n");
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
// PID state variables
double turnIntegral = 0.0;
double turnPrevError = 0.0;
double driveIntegral = 0.0;
double drivePrevError = 0.0;

double turnPID2(double targetHeading, int timeLimit = 0) {
  double kP = 0.37;
  double kI = 0.012;
  double kD = 3;
  
  turnIntegral = 0.0;
    turnPrevError = 0.0;  
  int settledTime = 0;
  const int requiredSettledTime = 50;
  int elapsedTime = 0;
  
  // Initialize prevError to avoid derivative spike on first iteration
  double currentHeading = InertialSensor.heading();
  
  double initialError = targetHeading - currentHeading;
  while (initialError > 180) initialError -= 360;
  while (initialError < -180) initialError += 360;
  turnPrevError = initialError;
  
  while (settledTime < requiredSettledTime && (timeLimit == 0 || elapsedTime < timeLimit)) {
    double currentHeading = InertialSensor.heading();
    
    double error = targetHeading - currentHeading;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    
    if (fabs(error) < 20) {
      turnIntegral += error;
    } else {
      turnIntegral = 0;
    }
    
    if (turnIntegral > 50) turnIntegral = 50;
    if (turnIntegral < -50) turnIntegral = -50;
    
    double derivative = error - turnPrevError;
    double power = (kP * error) + (kI * turnIntegral) + (kD * derivative);
    
    if (power > 12.0) power = 12.0;
    if (power < -12.0) power = -12.0;
    
    if (fabs(power) < 1.7 && fabs(error) > 1.0) {
      power = (power > 0) ? 1.7 : -1.7;
    }

    if(fabs(error) < 1.0){
      power = 0;
      turnIntegral = 0;
      settledTime += 5;
    } else {
      settledTime = 0;
    }
    
    rightDrive.spin(forward, power, volt);
    leftDrive.spin(reverse, power, volt);
    
    turnPrevError = error;
    printf("Error %.2f, power %.2f, heading %.2f (raw: %.2f), target %.2f, settled %dms\n", error, power, currentHeading, InertialSensor.heading(), targetHeading, settledTime);
    
    wait(5, msec);
    elapsedTime += 5;
  }
  
  // Stop the motors when done
  leftDrive.stop(brake);
  rightDrive.stop(brake);
  
  return 0.0;
}



void setVelocity(int velocity) {

    leftDrive.setVelocity(velocity, percent);
    rightDrive.setVelocity(velocity, percent);

  }

void drive(int degreeNum, int dir) {
  if(dir == 0){
    leftDrive.spinFor(forward, degreeNum, degrees, false);
    rightDrive.spinFor(forward, degreeNum, degrees);
  }
  else{
    leftDrive.spinFor(reverse, degreeNum, degrees, false);
    rightDrive.spinFor(reverse, degreeNum, degrees);}
  }

void turn(int degreeNum, int dir){

  if(dir == 0){
    leftDrive.spinFor(forward, degreeNum, degrees, false);
    rightDrive.spinFor(reverse, degreeNum, degrees);
  }
  else{
    leftDrive.spinFor(reverse, degreeNum, degrees, false);
    rightDrive.spinFor(forward, degreeNum, degrees);}

}
  double prevDistance =0;
double trapezoidsAreYucky(double currentPosition, double targetDistance, double maxVelocity, double acceleration) {
  
  // Using: d = v² / (2a)
  double accelDistance = (maxVelocity * maxVelocity) / (2.0 * acceleration);
  double decelDistance = accelDistance*2;  // Symmetric profile
  double distToTarget = fabs(targetDistance) - fabs(currentPosition);
  /*
  if (accelDistance + decelDistance > fabs(targetDistance)) {
    // Triangular: v_peak = sqrt(distance * acceleration)
    double peakVelocity = sqrt(fabs(targetDistance) * acceleration);
    maxVelocity = peakVelocity;
    accelDistance = fabs(targetDistance) / 2.0;
    decelDistance = fabs(targetDistance) / 2.0;
  }
  */
  double cruiseDistance = fabs(targetDistance) - accelDistance - decelDistance;
  double absPosition = fabs(currentPosition);
  double velocity = 0;
    double currentVelocity =  (absPosition-prevDistance)*0.2;

  // Minimum velocity to overcome static friction (mm/s)
  const double MIN_VELOCITY = 100.0;
  
  // Determine phase and calculate velocity
  if (absPosition < accelDistance) {
    // ACCELERATION: v = sqrt(2 * a * d)
    velocity = sqrt(2.0 * acceleration * absPosition);
    if (velocity > maxVelocity) velocity = maxVelocity;
    if (velocity < MIN_VELOCITY) velocity = MIN_VELOCITY;  // Minimum to start moving
    printf("ACCELERATING, currentVelocity: %.2f mm/s, targetVelocity: %.2f mm/s\n", currentVelocity, velocity);
  } 
   if (absPosition > accelDistance && absPosition < accelDistance + cruiseDistance) {
    // CRUISE: constant max velocity
    velocity = maxVelocity;
    printf("CRUISING, currentVelocity: %.2f mm/s, targetVelocity: %.2f mm/s\n", currentVelocity, velocity);
  } 

   if (absPosition > accelDistance + cruiseDistance) {
    // DECELERATION: v = sqrt(2 * a * distance_remaining)
    double distanceRemaining = fabs(targetDistance) - absPosition;
    velocity = sqrt(2.0 * acceleration * distanceRemaining)/4.0;

    if (velocity < 0) velocity = 0;  // Minimum to keep moving
    printf("DECELERATING, currentVelocity: %.2f mm/s, targetVelocity: %.2f mm/s\n", currentVelocity, velocity);
  }
  // printf("Current Position: %.2f mm, Distance to Target: %.2f mm\n, accel", currentPosition, distToTarget);
  // printf("Accel Distance: %.2f mm, Cruise Distance: %.2f mm, Decel Distance: %.2f mm\n", accelDistance, cruiseDistance, decelDistance);
  // printf("Calculated Velocity: %.2f mm/s\n", velocity);

  prevDistance = absPosition;

  
  return velocity;  // Returns mm/s
  wait(20, msec); 
}

void trapDrive(double targetDistance, double maxVelocity, double acceleration, bool reverse = false) {
  leftDrive.setPosition(0, degrees);
  rightDrive.setPosition(0, degrees);
  while(true) {
        // Get average position from all 6 motors for accuracy
        double leftAvg = (fabs(frontLeftDrive.position(degrees)) + 
                         fabs(middleLeftDrive.position(degrees)) + 
                         fabs(backLeftDrive.position(degrees))) / 3.0;
        double rightAvg = (fabs(frontRightDrive.position(degrees)) + 
                          fabs(middleRightDrive.position(degrees)) + 
                          fabs(backRightDrive.position(degrees))) / 3.0;
        double motorDegrees = (leftAvg + rightAvg) / 2.0;
        // printf("Front Left Drive Degrees: %.2f\n", tempPosition);
        double currentPosition_mm = degreesToMM(motorDegrees);
        // printf("Current Position in mm: %.2f\n", currentPosition_mm);
        
        // Debug: print raw motor degrees
       // printf("Motor degrees: %.2f, Converted to mm: %.2f\n", motorDegrees, currentPosition_mm);
        
        if (currentPosition_mm >= targetDistance) break;
        
        double targetVelocity_mmps = trapezoidsAreYucky(currentPosition_mm, targetDistance, maxVelocity, acceleration);
        if(!reverse){
          leftDrive.spin(forward, -targetVelocity_mmps*0.06, volt); 
          rightDrive.spin(forward, -targetVelocity_mmps*0.06, volt);
        }
        if(reverse){
          leftDrive.spin(forward, targetVelocity_mmps*0.06, volt); 
          rightDrive.spin(forward, targetVelocity_mmps*0.06, volt);
        }
        wait(10, msec);
      }
      
      // Stop when button released or target reached
      leftDrive.stop(brake);
      rightDrive.stop(brake);
}


void autonomous(void) {
InertialSensor.setHeading(90, degrees);
trapDrive(622, 300, 350);
wait(0.5, sec);
turnPID2(180);
trapDrive(100, 300, 350,true);
  unloader.set(true);

wait(1, sec);
lowBlockTrack.spin(forward, 12, volt);
trapDrive(220, 700, 500);
wait(2, sec);
trapDrive(100, 500, 580, true);
wait(0.5, sec);
turnPID2(225);
trapDrive(335, 300, 350,true);
wait(0.5, sec);
turnPID2(180);
wait(0.5, sec);
lowBlockTrack.stop();
unloader.set(false);
trapDrive(600, 300, 350, true);
wait(0.5, sec);
turnPID2(180);
wait(0.5, sec);
trapDrive(800, 300, 350, true);
wait(0.5, sec);
turnPID2(165);  
leftDrive.stop();
rightDrive.stop();

}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
double tempPosition;
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    if(Controller.ButtonB.pressing()){
      // printf("Left motors: FL=%.1f, ML=%.1f, BL=%.1f\n", 
      //        frontLeftDrive.position(degrees), 
      //        middleLeftDrive.position(degrees), 
      //        backLeftDrive.position(degrees));
      // // printf("Right motors: FR=%.1f, MR=%.1f, BR=%.1f\n", 
      //        frontRightDrive.position(degrees), 
      //        middleRightDrive.position(degrees), 
      //        backRightDrive.position(degrees));
      //        wait(500, msec);
    }
    
    // Example: Test trapezoidal motion when ButtonA is pressed
    if(Controller.ButtonY.pressing()){
      // Reset position
      leftDrive.setPosition(0, degrees);
      rightDrive.setPosition(0, degrees);
      frontLeftDrive.setPosition(0, degrees);
    }
     
    while(Controller.ButtonA.pressing()) {
        // Get average position from all 6 motors for accuracy
        double leftAvg = (fabs(frontLeftDrive.position(degrees)) + 
                         fabs(middleLeftDrive.position(degrees)) + 
                         fabs(backLeftDrive.position(degrees))) / 3.0;
        double rightAvg = (fabs(frontRightDrive.position(degrees)) + 
                          fabs(middleRightDrive.position(degrees)) + 
                          fabs(backRightDrive.position(degrees))) / 3.0;
        //double motorDegrees = (leftAvg + rightAvg) / 2.0;
         tempPosition = fabs(frontLeftDrive.position(degrees));
        // printf("Front Left Drive Degrees: %.2f\n", tempPosition);
        double currentPosition_mm = degreesToMM(tempPosition);
        // printf("Current Position in mm: %.2f\n", currentPosition_mm);
        
        // Debug: print raw motor degrees
       // printf("Motor degrees: %.2f, Converted to mm: %.2f\n", motorDegrees, currentPosition_mm);
        
        if (currentPosition_mm >= 1000) break;
        
        double targetVelocity_mmps = trapezoidsAreYucky(currentPosition_mm, 1000, 300, 350);
        leftDrive.spin(forward, targetVelocity_mmps*0.012, volt); 
        rightDrive.spin(forward, targetVelocity_mmps*0.012, volt);
        wait(10, msec);
      }
      
      // Stop when button released or target reached
      leftDrive.stop(brake);
      rightDrive.stop(brake);
      

    }
    
    // printf("Front Left Drive Degrees: %.2f\n", tempPosition);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
 // Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
