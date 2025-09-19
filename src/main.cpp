/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jokin                                                     */
/*    Created:      9/9/2025, 4:42:40 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

competition Competition;

brain Brain;
controller Controller;

/* Drive Base Motors (PLACEHOLDER PORT NUMBERS)*/
motor frontLeftDrive = motor(PORT1, ratio6_1, true);
motor middleLeftDrive = motor(PORT2, ratio6_1, true);
motor backLeftDrive = motor(PORT3, ratio6_1, true);

motor_group leftDrive = motor_group(frontLeftDrive, middleLeftDrive, backLeftDrive);

motor frontRightDrive = motor(PORT4, ratio6_1, false);
motor middleRightDrive = motor(PORT5, ratio6_1, false);
motor backRightDrive = motor(PORT6, ratio6_1, false);
motor_group rightDrive = motor_group(frontRightDrive, middleRightDrive, backRightDrive);


/* Functions */

void updateDriveSpeed(void){ //Split Arcade Drive Control, controlled with voltage

  leftDrive.spin(forward, Controller.Axis3.position()*0.12 + Controller.Axis1.position()*0.06  , volt);
  rightDrive.spin(forward,   Controller.Axis3.position()*0.12 - Controller.Axis1.position()*0.09 , volt);

} 

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after th e V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

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

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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

void usercontrol(void) {
  while (1) {
    

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol); 

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
