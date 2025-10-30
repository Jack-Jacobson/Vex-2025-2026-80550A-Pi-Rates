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

distance topBlockDist = distance(PORT20);
distance midBlockDist = distance(PORT19);

/* Left Drive Base Motors */
motor frontLeftDrive = motor(PORT1, ratio6_1, true);
motor backLeftDrive = motor(PORT2, ratio6_1, false);
motor_group leftDrive = motor_group(frontLeftDrive, backLeftDrive);

/* Right Drive Base Motors */
motor frontRightDrive = motor(PORT3, ratio6_1, false);
motor backRightDrive = motor(PORT4, ratio6_1, true);
motor_group rightDrive = motor_group(frontRightDrive, backRightDrive);

/* Block Track Motors */
motor blockTrack1 = motor(PORT13, gearSetting::ratio18_1, true);
motor blockTrack2 = motor(PORT14, gearSetting::ratio18_1, false);
motor blockTrack3 = motor(PORT15, gearSetting::ratio18_1, true);
motor blockTrack4 = motor(PORT16, gearSetting::ratio18_1, true);

/* Unloader */
digital_out unloader = digital_out(Brain.ThreeWirePort.A);

/* Variable Declerations */
int screen = 0; //Used for screen display,0 = Main, 1 = Auton Selection, 2 = Settings, 3 = Motor Information

/* Motor Declerations */
int motorStatusTimer = 0;//Timer for motor status updates
bool frontLeftStatus = frontLeftDrive.installed();
bool backLeftStatus = backLeftDrive.installed();
bool frontRightStatus = frontRightDrive.installed();
bool backRightStatus = backRightDrive.installed();
bool blockTrack1Status = blockTrack1.installed();
bool blockTrack2Status = blockTrack2.installed();
bool blockTrack3Status = blockTrack3.installed();

bool blockTrack4Status = blockTrack4.installed();
bool distanceStatus = topBlockDist.installed();
bool distance2Status = midBlockDist.installed();

/* Functions */

void updateDriveSpeed(void){ //Split Arcade Drive Control, controlled with voltage

  double forwardVal = -Controller.Axis1.position();
  double turnVal = -Controller.Axis3.position();
  double leftPower = forwardVal + turnVal;
  double rightPower = forwardVal - turnVal;

  double leftVoltage = leftPower * 0.12;
  double rightVoltage = rightPower * 0.12;

  frontRightDrive.spin(vex::forward, rightVoltage, vex::voltageUnits::volt);
  backRightDrive.spin(vex::forward, rightVoltage, vex::voltageUnits::volt);
  frontLeftDrive.spin(vex::forward, leftVoltage, vex::voltageUnits::volt);
  backLeftDrive.spin(vex::forward, leftVoltage, vex::voltageUnits::volt);

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

void driveForward(int degreeNum) {

    leftDrive.spinFor(reverse, degreeNum, degrees, false);
    rightDrive.spinFor(degreeNum, degrees);

  }

void driveReverse(int degreeNum) {

    leftDrive.spinFor(forward, degreeNum, degrees, false);
    rightDrive.spinFor(reverse, degreeNum, degrees);

  }

void setVelocity(int velocity) {

    leftDrive.setVelocity(velocity, percent);
    rightDrive.setVelocity(velocity, percent);

  }

bool loadingBlocks = false;

void loadBalls(){
  while (loadingBlocks) {
    blockTrack1.spin(forward, 12, volt);
    if(midBlockDist.objectDistance(mm) >= 150){
      blockTrack2.spin(forward, 12, volt);
    } else if(midBlockDist.objectDistance(mm) < 150 && topBlockDist.objectDistance(mm) < 150){
      blockTrack2.stop(hold);
    }
    if(topBlockDist.objectDistance(mm) >= 150){
      blockTrack3.spin(forward, 12, volt);
    } else if(topBlockDist.objectDistance(mm) < 150){
      blockTrack3.stop(hold);
    }
    wait(10, msec); 
  }
  blockTrack1.stop();
  blockTrack2.stop();
  blockTrack3.stop();
  blockTrack4.stop();
}

void extakeBalls(){}

void scoreTop(){
  blockTrack1.spin(forward, 12, volt);
  blockTrack2.spin(forward, 12, volt);
  blockTrack3.spin(forward, 12, volt);
  blockTrack4.spin(forward, 12, volt);
}

void turn(int direction, int degreeNum) {

    if (direction == 0) {

      leftDrive.spinFor(degreeNum, degrees, false);
      rightDrive.spinFor(degreeNum, degrees);

    } else {

      leftDrive.spinFor(reverse, degreeNum, degrees, false);
      rightDrive.spinFor(reverse, degreeNum, degrees);
    }

  }


void brainUI(void){

  motorStatusTimer+=5;

  if (motorStatusTimer>=500){ 
    motorStatusTimer=0;
    frontLeftStatus = frontLeftDrive.installed();
    backLeftStatus = backLeftDrive.installed();
    frontRightStatus = frontRightDrive.installed();
    backRightStatus = backRightDrive.installed();
    blockTrack1Status = blockTrack1.installed();
    blockTrack2Status = blockTrack2.installed();
    blockTrack3Status = blockTrack3.installed();
  blockTrack4Status = blockTrack4.installed();
  distanceStatus = topBlockDist.installed();
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


      Brain.Screen.setPenColor(frontLeftStatus ? white : red);
      Brain.Screen.printAt(10, 20, false, "Front Left Drive (Port 1): %s", frontLeftStatus ? "Connected" : "Disconnected");
      if (frontLeftStatus) {
        double flTemp = frontLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(flTemp > 50 ? (flTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 20, false, "%.1f°C", flTemp);
      }

      Brain.Screen.setPenColor(backLeftStatus ? white : red);
      Brain.Screen.printAt(10, 40, false, "Back Left Drive (Port 2): %s", backLeftStatus ? "Connected" : "Disconnected");
      if (backLeftStatus) {
        double blTemp = backLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(blTemp > 50 ? (blTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 40, false, "%.1f°C", blTemp);
      }

      Brain.Screen.setPenColor(frontRightStatus ? white : red);
      Brain.Screen.printAt(10, 60, false, "Front Right Drive (Port 3): %s", frontRightStatus ? "Connected" : "Disconnected");
      if (frontRightStatus) {
        double frTemp = frontRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(frTemp > 50 ? (frTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 60, false, "%.1f°C", frTemp);
      }

      Brain.Screen.setPenColor(backRightStatus ? white : red);
      Brain.Screen.printAt(10, 80, false, "Back Right Drive (Port 4): %s", backRightStatus ? "Connected" : "Disconnected");
      if (backRightStatus) {
        double brTemp = backRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(brTemp > 50 ? (brTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 80, false, "%.1f°C", brTemp);
      }

      Brain.Screen.setPenColor(blockTrack1Status ? white : red);
      Brain.Screen.printAt(10, 100, false, "Block Track 1 (Port 13): %s", blockTrack1Status ? "Connected" : "Disconnected");
      if (blockTrack1Status) {
        double bt1Temp = blockTrack1.temperature(celsius);
        Brain.Screen.setPenColor(bt1Temp > 50 ? (bt1Temp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 100, false, "%.1f°C", bt1Temp);
      }

      Brain.Screen.setPenColor(blockTrack2Status ? white : red);
      Brain.Screen.printAt(10, 120, false, "Block Track 2 (Port 14): %s", blockTrack2Status ? "Connected" : "Disconnected");
      if (blockTrack2Status) {
        double bt2Temp = blockTrack2.temperature(celsius);
        Brain.Screen.setPenColor(bt2Temp > 50 ? (bt2Temp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 120, false, "%.1f°C", bt2Temp);
      }

      Brain.Screen.setPenColor(blockTrack3Status ? white : red);
      Brain.Screen.printAt(10, 140, false, "Block Track 3 (Port 15): %s", blockTrack3Status ? "Connected" : "Disconnected");
      if (blockTrack3Status) {
        double bt3Temp = blockTrack3.temperature(celsius);
        Brain.Screen.setPenColor(bt3Temp > 50 ? (bt3Temp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 140, false, "%.1f°C", bt3Temp);
      }

      Brain.Screen.setPenColor(blockTrack4Status ? white : red);
      Brain.Screen.printAt(10, 160, false, "Block Track 4 (Port 16): %s", blockTrack4Status ? "Connected" : "Disconnected");
      if (blockTrack4Status) {
        double bt4Temp = blockTrack4.temperature(celsius);
        Brain.Screen.setPenColor(bt4Temp > 50 ? (bt4Temp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 160, false, "%.1f°C", bt4Temp);
      }

      // Top Distance Sensor Info
      Brain.Screen.setPenColor(distanceStatus ? white : red);
      Brain.Screen.printAt(10, 180, false, "Top Distance Sensor (Port 20): %s", distanceStatus ? "Connected" : "Disconnected");
      if (distanceStatus) {
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(420, 180, false, "%.1f mm", topBlockDist.objectDistance(mm));
      }

      // Mid Distance Sensor Info
      Brain.Screen.setPenColor(distance2Status ? white : red);
      Brain.Screen.printAt(10, 200, false, "Mid Distance Sensor (Port 19): %s", distance2Status ? "Connected" : "Disconnected");
      if (distance2Status) {
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(420, 200, false, "%.1f mm", midBlockDist.objectDistance(mm));
      }

      Brain.Screen.setPenColor(white);
      drawButton(5 , 212, 60, 20, "Back", 0);
      break;

    }
  
  Brain.Screen.render();
  wait(5, msec);

}

void pre_auton(void) {
  unloader.set(false);
}

void autonomous(void) {
  loadingBlocks = true;
  thread ballLoaderThread(loadBalls);
  setVelocity(30);
  driveForward(1100);
  setVelocity(20);  
  driveForward(500);
  wait(2, sec);
  setVelocity(40);
  driveForward(120);
  driveReverse(800);
  loadingBlocks = false;
  turn(0, 600);
  driveReverse(1700);
  turn(0, 625);
  leftDrive.setTimeout(1.5, sec);
  rightDrive.setTimeout(1.5, sec);
  driveReverse(800);
  driveForward(30);
  scoreTop();  
}


void usercontrol(void) {
  while (1) {
    
    updateDriveSpeed();

    
    brainUI();

    if(Controller.ButtonR2.pressing()){
      blockTrack1.spin(forward, 12, volt);
      blockTrack2.spin(forward, 12, volt);
      blockTrack3.spin(forward, 12, volt);
      blockTrack4.spin(forward, 12, volt);
    }
    else if(Controller.ButtonL2.pressing()){
      blockTrack1.spin(reverse, 12, volt);
      blockTrack2.spin(reverse, 12, volt);
      blockTrack3.spin(reverse, 12, volt);
      blockTrack4.spin(reverse, 12, volt);
    }
    else if(Controller.ButtonR1.pressing()){
      blockTrack1.spin(forward, 12, volt);
      blockTrack2.spin(forward, 12, volt);
      blockTrack3.spin(reverse, 12, volt);
      blockTrack4.spin(reverse, 12, volt);
    }
    else if(Controller.ButtonL1.pressing()){
      blockTrack1.spin(forward, 12, volt);
      if(midBlockDist.objectDistance(mm)>=150){
        blockTrack2.spin(forward, 12, volt);
      }
      else if(midBlockDist.objectDistance(mm)<150 && topBlockDist.objectDistance(mm)<150){
        blockTrack2.stop(hold);
            }
      if(topBlockDist.objectDistance(mm)>=150){
        blockTrack3.spin(forward, 12, volt);
      }
      else if(topBlockDist.objectDistance(mm)<150){
        blockTrack3.stop(hold);
        }
      }
      else{
        blockTrack1.stop();
        blockTrack2.stop();
        blockTrack3.stop();
        blockTrack4.stop();
      }

      if(Controller.ButtonX.PRESSED){

      unloader.set(!unloader.value());

      }
    wait(10, msec); // Sleep the task for a short amount of time to
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
