/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jokin                                                     */
/*    Created:      9/9/2025, 4:42:40 PM                                      */
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

/* Drive Base Motors (PLACEHOLDER PORT NUMBERS)*/
motor frontLeftDrive = motor(PORT1, ratio6_1, true);
motor middleLeftDrive = motor(PORT2, ratio6_1, true);
motor backLeftDrive = motor(PORT3, ratio6_1, true);
motor_group leftDrive = motor_group(frontLeftDrive, middleLeftDrive, backLeftDrive);

motor frontRightDrive = motor(PORT4, ratio6_1, false);
motor middleRightDrive = motor(PORT5, ratio6_1, false);
motor backRightDrive = motor(PORT6, ratio6_1, false);
motor_group rightDrive = motor_group(frontRightDrive, middleRightDrive, backRightDrive);

/* Pnumatic Definitions (PLACEHOLDER PORT NUMBERS) */
digital_out armPiston = digital_out(Brain.ThreeWirePort.A);

/* Variable Declerations */
int screen = 0; //Used for screen display,0 = Main, 1 = Auton Selection, 2 = Settings, 3 = Motor Information

int motorStatusTimer = 0;//Timer for motor status updates
bool leftFrontStatus = frontLeftDrive.installed(), leftMiddleStatus = middleLeftDrive.installed(), leftBackStatus = backLeftDrive.installed(), rightFrontStatus = frontRightDrive.installed(), rightMiddleStatus = middleRightDrive.installed(), rightBackStatus = backRightDrive.installed(); // Motor status variables


/* Functions */

void updateDriveSpeed(void){ //Split Arcade Drive Control, controlled with voltage

  leftDrive.spin(forward, Controller.Axis3.position()*0.12 + Controller.Axis1.position()*0.06  , volt);
  rightDrive.spin(forward,   Controller.Axis3.position()*0.12 - Controller.Axis1.position()*0.09 , volt);

} 

void drawButton(int x, int y, int w, int h, std::string t, int destination) {
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawRectangle(x, y, w, h);

    int textWidth = Brain.Screen.getStringWidth(t.c_str());
    int textHeight = Brain.Screen.getStringHeight(t.c_str());
    int textX = x + (w - textWidth) / 2;
    int textY = y + (h + textHeight) / 2;
    Brain.Screen.printAt(textX, textY, false, t.c_str());

    if (Brain.Screen.pressing()) {
        int touchX = Brain.Screen.xPosition();
        int touchY = Brain.Screen.yPosition();

        if (touchX >= x && touchX <= x + w && touchY >= y && touchY <= y + h) {
            screen = destination;
            Brain.Screen.clearScreen();
            return;
        }
    }
}// Draws a button at specified location with text, changes screen variable to destination if pressed (MADE IN 24-25 SEASON BY ME)

void brainUI(void){

  motorStatusTimer+=5;

  if (motorStatusTimer>=500){ 
    
    motorStatusTimer=0;

    leftFrontStatus = frontLeftDrive.installed();
    leftMiddleStatus = middleLeftDrive.installed();
    leftBackStatus = backLeftDrive.installed();
    rightFrontStatus = frontRightDrive.installed();
    rightMiddleStatus = middleRightDrive.installed();
    rightBackStatus = backRightDrive.installed();
  
  }
  Brain.Screen.clearScreen();

  switch(screen){
    case 0: //Main Screen
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);

      drawButton(35, 40, 200, 160, "Auton", 1);
      drawButton(245, 40, 200, 160, "Status Check", 2);
      break;
      
    case 1: //Auton Selection Screen
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);

      drawButton(5 , 212, 60, 20, "Back", 0);
      break;

    case 2: //Motor Information
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);

      Brain.Screen.setPenColor(leftFrontStatus ? white : red);
      Brain.Screen.printAt(10, 50, false, "Front Left Drive: %s", leftFrontStatus ? "Connected" : "Disconnected");
      if (leftFrontStatus) {
        double flTemp = frontLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(flTemp > 50 ? (flTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 50, false, "%.1f°C", flTemp);
      }

      Brain.Screen.setPenColor(leftMiddleStatus ? white : red);
      Brain.Screen.printAt(10, 70, false, "Middle Left Drive: %s", leftMiddleStatus ? "Connected" : "Disconnected");
      if (leftMiddleStatus) {
        double mlTemp = middleLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(mlTemp > 50 ? (mlTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 70, false, "%.1f°C", mlTemp);
      }

      Brain.Screen.setPenColor(leftBackStatus ? white : red);
      Brain.Screen.printAt(10, 90, false, "Back Left Drive: %s", leftBackStatus ? "Connected" : "Disconnected");
      if (leftBackStatus) {
        double blTemp = backLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(blTemp > 50 ? (blTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 90, false, "%.1f°C", blTemp);
      }

      Brain.Screen.setPenColor(rightFrontStatus ? white : red);
      Brain.Screen.printAt(10, 110, false, "Front Right Drive: %s", rightFrontStatus ? "Connected" : "Disconnected");
      if (rightFrontStatus) {
        double frTemp = frontRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(frTemp > 50 ? (frTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 110, false, "%.1f°C", frTemp);
      }

      Brain.Screen.setPenColor(rightMiddleStatus ? white : red);
      Brain.Screen.printAt(10, 130, false, "Middle Right Drive: %s", rightMiddleStatus ? "Connected" : "Disconnected");
      if (rightMiddleStatus) {
        double mrTemp = middleRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(mrTemp > 50 ? (mrTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 130, false, "%.1f°C", mrTemp);
      }

      Brain.Screen.setPenColor(rightBackStatus ? white : red);
      Brain.Screen.printAt(10, 150, false, "Back Right Drive: %s", rightBackStatus ? "Connected" : "Disconnected");
      if (rightBackStatus) {
        double brTemp = backRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(brTemp > 50 ? (brTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 150, false, "%.1f°C", brTemp);
      }

      Brain.Screen.setPenColor(armPiston.value() ? white : red);
      Brain.Screen.printAt(10, 170, false, "Arm Piston: %s", armPiston.value() ? "Extended" : "Retracted");

      Brain.Screen.setPenColor(white);
      drawButton(5 , 212, 60, 20, "Back", 0);
      break;

    }
  
  Brain.Screen.render();
  wait(5, msec);

}

void pre_auton(void) {

  armPiston.set(false);//CHANGE IF INVERTED

}

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
    
    updateDriveSpeed();

    /*Arm Control*/
    static bool armExtended = false;
    if(Controller.ButtonA.pressing()){
      armExtended = !armExtended;
      armPiston.set(armExtended);
    }

    brainUI();

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
