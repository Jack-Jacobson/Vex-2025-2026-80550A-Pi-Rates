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

motor frontLeftDrive = motor(PORT4, ratio6_1, false);
motor middleLeftDrive = motor(PORT5, ratio6_1, false);
motor backLeftDrive = motor(PORT3, ratio6_1, false);
motor_group leftDrive = motor_group(frontLeftDrive, middleLeftDrive, backLeftDrive);

motor frontRightDrive = motor(PORT2, ratio6_1, true);
motor middleRightDrive = motor(PORT6, ratio6_1, true);
motor bottomRightDrive = motor(PORT1, ratio6_1, true);
motor_group rightDrive = motor_group(frontRightDrive, middleRightDrive, bottomRightDrive);

motor lowBlockTrack = motor(PORT14, ratio6_1, true);
motor highBlockTrack = motor(PORT15, ratio6_1, true);

distance leftWallDist = distance(PORT19);

//Radio in 21

digital_out unloader = digital_out(Brain.ThreeWirePort.A);
digital_out descore = digital_out(Brain.ThreeWirePort.H);

int screen = 0;
int currentAuton = 0; // -1 = no auton, 0 = right-side, 1 = drive forward, 2 = left-side, 3 = fill low

int motorStatusTimer = 0;
bool frontLeftStatus = frontLeftDrive.installed();
bool middleLeftStatus = middleLeftDrive.installed();
bool backLeftStatus = backLeftDrive.installed();
bool frontRightStatus = frontRightDrive.installed();
bool middleRightStatus = middleRightDrive.installed();
bool backRightStatus = bottomRightDrive.installed();
bool lowBlockTrackStatus = lowBlockTrack.installed();
bool highBlockTrackStatus = highBlockTrack.installed();
bool leftWallDistStatus = leftWallDist.installed();
bool inertialStatus = InertialSensor.installed();
bool calibrated = false;
bool spin1 = true;
bool spin2 = true;
bool spin3 = true;
bool spin4 = true;

double inchesPerTick = 0.0279;

double robotX = 0.0;
double robotY = 0.0;
double robotHeading = 0.0;
double prevLeftTicks = 0.0;
double prevRightTicks = 0.0;

// PID state variables
double turnIntegral = 0.0;
double turnPrevError = 0.0;
double driveIntegral = 0.0;
double drivePrevError = 0.0;

void updateRobotPosition(void) {
  double leftTicks = leftDrive.position(vex::rotationUnits::raw);
  double rightTicks = rightDrive.position(vex::rotationUnits::raw);
  
  double deltaLeftTicks = leftTicks - prevLeftTicks;
  double deltaRightTicks = rightTicks - prevRightTicks;
  
  double avgDeltaTicks = (deltaLeftTicks + deltaRightTicks) / 2.0;
  double distanceTraveled = avgDeltaTicks * inchesPerTick;
  
  robotHeading = InertialSensor.heading();
  
  double headingRad = robotHeading * M_PI / 180.0;

  robotX += distanceTraveled * sin(headingRad);
  robotY += distanceTraveled * cos(headingRad);
  
  prevLeftTicks = leftTicks;
  prevRightTicks = rightTicks;
}


double turnPID(double targetHeading) {
  double kP = 0.3;
  double kI = 0.005;
  double kD = 0.9;
  
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
  
  if (fabs(power) < 1.5 && fabs(error) > 1.0) {
    power = (power > 0) ? 1.5 : -1.5;
  }

  if(fabs(error)<1.0){
    power =0;
    turnIntegral =0;
  }
  
  turnPrevError = error;
  printf("Error %.2f, power %.2f, heading %.2f, target %.2f\n", error, power, InertialSensor.heading(), targetHeading);

  return power;
}

double driveForwardPID(double targetInches) {
  double kP = 0.5;
  double kI = 0.005;
  double kD = 0.3;
  
  double targetDegrees = targetInches / ((3.25 * M_PI) / 360.0);
  
  double avgPosition = (leftDrive.position(degrees) + rightDrive.position(degrees)) / 2.0;
  double error = targetDegrees - avgPosition;
  
  if (fabs(error) < 100) {
    driveIntegral += error;
  } else {
    driveIntegral = 0;
  }
  
  if (driveIntegral > 100) driveIntegral = 100;
  if (driveIntegral < -100) driveIntegral = -100;
  
  double derivative = error - drivePrevError;
  double power = (kP * error) + (kI * driveIntegral) + (kD * derivative);
  
  if (power > 12.0) power = 12.0;
  if (power < -12.0) power = -12.0;
  
  if (fabs(power) < 2.0 && fabs(error) > 5.0) {
    power = (power > 0) ? 2.0 : -2.0;
  }
  
  drivePrevError = error;
  
  return power;
}

void stopDrive(brakeType mode = brake) {
  leftDrive.stop(mode);
  rightDrive.stop(mode);
}

void setDriveVelocity(int velocity) {
  leftDrive.setVelocity(velocity, percent);
  rightDrive.setVelocity(velocity, percent);
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
}

void drawAutonButton(int x, int y, int w, int h, std::string t, int autonNum, color textColor = white) {
    Brain.Screen.setFillColor(black);
    
    if (currentAuton == autonNum) {
        Brain.Screen.setPenColor(blue);
        Brain.Screen.setPenWidth(4);
    } else {
        Brain.Screen.setPenColor(white);
        Brain.Screen.setPenWidth(1);
    }
    
    Brain.Screen.drawRectangle(x, y, w, h);
    Brain.Screen.setPenWidth(1);

    int textWidth = Brain.Screen.getStringWidth(t.c_str());
    int textHeight = Brain.Screen.getStringHeight(t.c_str());
    int textX = x + (w - textWidth) / 2;
    int textY = y + (h + textHeight) / 2;
    
    Brain.Screen.setPenColor(textColor);
    Brain.Screen.printAt(textX, textY, false, t.c_str());

    if (Brain.Screen.pressing()) {
        int touchX = Brain.Screen.xPosition();
        int touchY = Brain.Screen.yPosition();

        if (touchX >= x && touchX <= x + w && touchY >= y && touchY <= y + h) {
            currentAuton = autonNum;
            
            // Update controller display
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            if (currentAuton == -1) {
                Controller.Screen.print("Auton: No Auton");
            } else if (currentAuton == 0) {
                Controller.Screen.print("Auton: Right-Side");
            } else if (currentAuton == 1) {
                Controller.Screen.print("Auton: Drive Fwd");
            } else if (currentAuton == 2) {
                Controller.Screen.print("Auton: Left-Side");
            } else if (currentAuton == 3) {
                Controller.Screen.print("Auton: Fill Low");
            }
            
            wait(100, msec);
            return;
        }
    }
}

void driveForward(int degreeNum) {

    leftDrive.spinFor(reverse, degreeNum, degrees, false);
    rightDrive.spinFor(degreeNum, degrees);

  }
void setVelocity(int velocity) {

    leftDrive.setVelocity(velocity, percent);
    rightDrive.setVelocity(velocity, percent);

  }
  
void updateDriveSpeed(void){
  static double tempTargetHeading = -1;
  
 if (tempTargetHeading < 0) {
    tempTargetHeading = InertialSensor.heading();
  }
  
  printf("Temp Target Heading: %.2f\n", tempTargetHeading);
  double forwardVal = -Controller.Axis3.position();
  if(Controller.Axis1.position()<-50){
    tempTargetHeading -=0.1;
  }
  if(Controller.Axis1.position()>50){
    tempTargetHeading +=0.1;
  }
  
  // Normalize target heading to 0-360
  while (tempTargetHeading >= 360) tempTargetHeading -= 360;
  while (tempTargetHeading < 0) tempTargetHeading += 360;
  
  double turnPower = turnPID(tempTargetHeading);
  double forwardVoltage = forwardVal * 0.12;
  
  double leftVoltage = forwardVoltage + turnPower;
  double rightVoltage = forwardVoltage - turnPower;

  frontRightDrive.spin(vex::forward, rightVoltage, vex::voltageUnits::volt);
  middleRightDrive.spin(vex::forward, rightVoltage, vex::voltageUnits::volt);
  bottomRightDrive.spin(vex::forward, rightVoltage, vex::voltageUnits::volt);
  frontLeftDrive.spin(vex::forward, leftVoltage, vex::voltageUnits::volt);
  middleLeftDrive.spin(vex::forward, leftVoltage, vex::voltageUnits::volt);
  backLeftDrive.spin(vex::forward, leftVoltage, vex::voltageUnits::volt);

} 

void driveTicks(double inches){
  double ticks = inches / inchesPerTick;
  leftDrive.spinFor(reverse, ticks, vex::rotationUnits::raw, false);
  rightDrive.spinFor(ticks, vex::rotationUnits::raw);
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
  int targetTurn = InertialSensor.angle()-degreeNum;
    leftDrive.spinFor(forward, degreeNum, degrees, false);
    rightDrive.spinFor(reverse, degreeNum, degrees);
  }
  else{
    int targetTurn = InertialSensor.angle()-degreeNum;
    leftDrive.spinFor(reverse, degreeNum, degrees, false);
    rightDrive.spinFor(forward, degreeNum, degrees);}

}
/*
void driveToPoint(int x, int y) {
  double deltaX = x - robotX;
  double deltaY = y - robotY;
  
  double distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  double targetAngle2 = atan2(deltaX, deltaY) * 180.0 / M_PI;
  
  if (targetAngle2 < 0) {
    targetAngle2 += 360;
  } 
  
  turnPID(targetAngle2, 1);

  setVelocity(80);
  driveTicks(distance);

  robotX = x;
  robotY = y;
  robotHeading = targetAngle2;
}*/

struct Point {
  double x;
  double y;
  double speed;
  double heading = -1;
  int flag = -1;
};
/** 
Point square[] = {
  {24, 0, 50}, 
  {24, 24, 50},  
  {0, 24, 50},   
  {0, 0, 50}
};

Point Auton1[] = {

  {12, 0, 40},
  {3, 0, 40}

};


bool loadingBlocks = false;

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
  

void drivePath(Point path[], int pathLength) {
  for (int i = 0; i < pathLength; i++) {
    
    if(path[i].flag == 0){
    loadBalls();
    } else if(path[i].flag == 1){
      
    }   
    else if(path[i].flag == 2){
      
    }
    
    setVelocity(path[i].speed);
    
    driveToPoint(path[i].x, path[i].y);
    
    if (path[i].heading >= 0) {
      turnPID(path[i].heading, 1);
      robotHeading = path[i].heading;
    }
    
  }
}
*/
void brainUI(void){

  motorStatusTimer+=5;

  if (motorStatusTimer>=500){ 
    motorStatusTimer=0;
    frontLeftStatus = frontLeftDrive.installed();
    middleLeftStatus = middleLeftDrive.installed();
    backLeftStatus = backLeftDrive.installed();
    frontRightStatus = frontRightDrive.installed();
    middleRightStatus = middleRightDrive.installed();
    backRightStatus = bottomRightDrive.installed();
    lowBlockTrackStatus = lowBlockTrack.installed();
    highBlockTrackStatus = highBlockTrack.installed();
    leftWallDistStatus = leftWallDist.installed();
    inertialStatus = InertialSensor.installed();
  }
  Brain.Screen.clearScreen();

  switch(screen){
    case 0:
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);

      drawButton(35, 40, 200, 160, "Auton", 1);
      drawButton(245, 40, 200, 160, "Status Check", 2);
      break;

    case 1:
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);
      Brain.Screen.setFont(prop30);

      drawAutonButton(10, 10, 460, 42, "No Auton", -1, red);
      drawAutonButton(10, 56, 460, 42, "Right-Side Auton", 0, white);
      drawAutonButton(10, 102, 460, 42, "Drive Forward Auton", 1, white);
      drawAutonButton(10, 148, 460, 42, "Left-Side Auton", 2, white);
      drawAutonButton(10, 194, 460, 42, "Fill Low", 3, white);
      
      Brain.Screen.setFont(prop20);
      drawButton(405, 212, 60, 20, "Back", 0);
      break;

    case 2:
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);
      Brain.Screen.setFont(prop20);

      Brain.Screen.setPenColor(frontLeftStatus ? white : red);
      Brain.Screen.printAt(10, 15, false, "Front Left (P4): %s", frontLeftStatus ? "OK" : "ERR");
      if (frontLeftStatus) {
        double flTemp = frontLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(flTemp > 50 ? (flTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 15, false, "%.1fC", flTemp);
      }

      Brain.Screen.setPenColor(middleLeftStatus ? white : red);
      Brain.Screen.printAt(10, 30, false, "Middle Left (P5): %s", middleLeftStatus ? "OK" : "ERR");
      if (middleLeftStatus) {
        double mlTemp = middleLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(mlTemp > 50 ? (mlTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 30, false, "%.1fC", mlTemp);
      }

      Brain.Screen.setPenColor(backLeftStatus ? white : red);
      Brain.Screen.printAt(10, 45, false, "Back Left (P3): %s", backLeftStatus ? "OK" : "ERR");
      if (backLeftStatus) {
        double blTemp = backLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(blTemp > 50 ? (blTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 45, false, "%.1fC", blTemp);
      }

      Brain.Screen.setPenColor(frontRightStatus ? white : red);
      Brain.Screen.printAt(10, 60, false, "Front Right (P2): %s", frontRightStatus ? "OK" : "ERR");
      if (frontRightStatus) {
        double frTemp = frontRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(frTemp > 50 ? (frTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 60, false, "%.1fC", frTemp);
      }

      Brain.Screen.setPenColor(middleRightStatus ? white : red);
      Brain.Screen.printAt(10, 75, false, "Middle Right (P6): %s", middleRightStatus ? "OK" : "ERR");
      if (middleRightStatus) {
        double mrTemp = middleRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(mrTemp > 50 ? (mrTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 75, false, "%.1fC", mrTemp);
      }

      Brain.Screen.setPenColor(backRightStatus ? white : red);
      Brain.Screen.printAt(10, 90, false, "Back Right (P1): %s", backRightStatus ? "OK" : "ERR");
      if (backRightStatus) {
        double brTemp = bottomRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(brTemp > 50 ? (brTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 90, false, "%.1fC", brTemp);
      }

      Brain.Screen.setPenColor(lowBlockTrackStatus ? white : red);
      Brain.Screen.printAt(10, 105, false, "Low Track (P11): %s", lowBlockTrackStatus ? "OK" : "ERR");
      if (lowBlockTrackStatus) {
        double lbtTemp = lowBlockTrack.temperature(celsius);
        Brain.Screen.setPenColor(lbtTemp > 50 ? (lbtTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 105, false, "%.1fC", lbtTemp);
      }

      Brain.Screen.setPenColor(highBlockTrackStatus ? white : red);
      Brain.Screen.printAt(10, 120, false, "High Track (P12): %s", highBlockTrackStatus ? "OK" : "ERR");
      if (highBlockTrackStatus) {
        double hbtTemp = highBlockTrack.temperature(celsius);
        Brain.Screen.setPenColor(hbtTemp > 50 ? (hbtTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 120, false, "%.1fC", hbtTemp);
      }

      Brain.Screen.setPenColor(leftWallDistStatus ? white : red);
      Brain.Screen.printAt(10, 135, false, "L Wall Dist (P19): %s", leftWallDistStatus ? "OK" : "ERR");
      if (leftWallDistStatus) {
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(380, 135, false, "%.0fmm", leftWallDist.objectDistance(mm));
      }

      Brain.Screen.setPenColor(inertialStatus ? white : red);
      Brain.Screen.printAt(240, 15, false, "IMU (P20): %s", inertialStatus ? "OK" : "ERR");

      Brain.Screen.setPenColor(white);
      drawButton(5, 212, 60, 20, "Back", 0);
      break;

    }
  
  Brain.Screen.render();
  wait(5, msec);

}

void pre_auton(void) {
  
  thread brainUIThread(brainUI);
  
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  if (currentAuton == -1) {
      Controller.Screen.print("Auton: No Auton");
  } else if (currentAuton == 0) {
      Controller.Screen.print("Auton: Right-Side");
  } else if (currentAuton == 1) {
      Controller.Screen.print("Auton: Drive Fwd");
  } else if (currentAuton == 2) {
      Controller.Screen.print("Auton: Left-Side");
  } else if (currentAuton == 3) {
      Controller.Screen.print("Auton: Fill Low");
  }
  
  if(!calibrated){
    int startTime = Brain.Timer.time(msec);
    InertialSensor.calibrate();
    while(InertialSensor.isCalibrating()){
            printf("Calibrating...\n");
            wait(100, msec);
          }
      InertialSensor.setHeading(180, degrees);
    int calibrationTime = Brain.Timer.time(msec) - startTime;
    printf("Calibrated in %dms\n", calibrationTime);
    calibrated = true;
  }
}
void autonomous(void) {
    unloader.set(false);
  /*
  loadingBlocks = true;
  thread ballLoaderThread(loadBalls);
  setVelocity(30);
  driveForward(1100);
  setVelocity(10);  
  driveForward(500);
  wait(2, sec);
  setVelocity(40);
  driveForward(120);
  driveReverse(800);
  loadingBlocks = false;
  setVelocity(20);
  turn(1, 600);
  setVelocity(40);
  driveReverse(1700);
  setVelocity(20);
  turn(1, 625);
  leftDrive.setTimeout(1.5, sec);
  rightDrive.setTimeout(1.5, sec);
  driveReverse(800);
  driveForward(30);
  scoreTop();  
  
 if(currentAuton == 0){
  
  descore.set(true);
  unloader.set(false);
  setVelocity(20);
  drive(200, 1);
  turn(100, 1);
  lowBlockTrack.spin(forward, 12, volt);
  highBlockTrack.setStopping(brake);
  drive(1400, 1);
  //After picking up first three balls:
  turn(370, 0);
  drive(760, 1);
  lowBlockTrack.spin(reverse,12, volt);
  wait(1.22, sec);//3 seconds for full unload, 1.22 for one
  lowBlockTrack.stop();
  //After attempting to score first ball:
    lowBlockTrack.stop();
  setVelocity(40);
  drive(2100, 0);
  setVelocity(20);
  turn(450, 0);
  drive(600, 0);
  setVelocity(20);
  turn(370, 0); 
  leftDrive.setTimeout(1, sec);
  rightDrive.setTimeout(1,sec);
  setVelocity(65);
  drive(1300, 0);
  //After lining up with goal:
  unloader.set(true);
  lowBlockTrack.spin(forward, 12, volt);
  setVelocity(50);
  drive(1600, 1);
  setVelocity(30);
  leftDrive.setTimeout(0.4, sec);
  rightDrive.setTimeout(0.4,sec);
  drive(400,1);
  wait(0.5, sec);
  setVelocity(40);
  drive(600, 0);
  /*
  if(leftWallDist.objectDistance(mm) <= 340 && leftWallDist.objectDistance(mm) >= 290){
    turn(45, 0);
  }
  else if(leftWallDist.objectDistance(mm) >= 410 && leftWallDist.objectDistance(mm) <= 460){
    turn(45, 1);
  }
  else if(leftWallDist.objectDistance(mm) <= 320 && leftWallDist.objectDistance(mm) >= 270){
    turn(90, 0);
  }
  else if(leftWallDist.objectDistance(mm) >= 460 && leftWallDist.objectDistance(mm) <= 510){
    turn(90, 1);
  }
  leftDrive.setTimeout(0.5, sec);
  rightDrive.setTimeout(0.5,sec);
  drive(900, 0);
  lowBlockTrack.spin(forward, 12, volt);
  highBlockTrack.spin(forward, 12, volt);

 }
 else if(currentAuton == 1){
   setVelocity(40);
   drive(100, 0);
 }
else if(currentAuton == 2){
 descore.set(true);
  unloader.set(false);
  setVelocity(20);
  drive(200, 1);
  turn(100, 1);
  blockTrack1.spin(forward, 12, volt);
  blockTrack2.spin(forward, 12, volt);
  blockTrack3.spin(forward, 12, volt);
  blockTrack4.spin(forward, 12, volt);
  blockTrack5.stop(brake);
  drive(1400, 1);
  //After picking up first three balls:
  turn(370, 0);
  drive(760, 1);
  /*
  blockTrack3.spin(reverse,12, volt);
  blockTrack2.spin(reverse,12, volt);
  blockTrack4.spin(reverse, 12, volt);
  blockTrack5.spin(reverse, 4, volt);
  blockTrack1.spin(reverse,5, volt);
  wait(1.22, sec);//3 seconds for full unload, 1.22 for one
  
  blockTrack1.stop();
  blockTrack2.stop();
  blockTrack3.stop();
  blockTrack4.stop();
  blockTrack5.stop();
  blockTrack1.spin(forward,12, volt);
  //After attempting to score first ball:
  blockTrack1.spin(forward, 12, volt);
  blockTrack3.spin(forward, 12, volt);
  blockTrack2.spin(forward, 12, volt);
  blockTrack4.spin(forward, 12, volt);
  setVelocity(40);
  drive(2100, 0);
  setVelocity(20);
  turn(450, 0);
  drive(600, 0);
  setVelocity(20);
  turn(370, 0); 
  leftDrive.setTimeout(1, sec);
  rightDrive.setTimeout(1,sec);
  setVelocity(65);
  drive(1300, 0);
  //After lining up with goal:
  unloader.set(true);
  blockTrack1.spin(forward, 12, volt);
  blockTrack2.spin(forward, 12, volt);
  blockTrack3.spin(forward, 12, volt);
  blockTrack4.spin(forward, 12, volt);
  blockTrack5.stop(brake);
  setVelocity(50);
  drive(1600, 1);
  setVelocity(30);
  leftDrive.setTimeout(0.4, sec);
  rightDrive.setTimeout(0.4,sec);
  drive(400,1);
  wait(0.5, sec);
  setVelocity(40);
  drive(600, 0);

}
else if(currentAuton == 3){
  
  descore.set(true);
  unloader.set(false);
  setVelocity(20);
  drive(200, 1);
  turn(100, 1);
  blockTrack1.spin(forward, 12, volt);
  blockTrack2.spin(forward, 12, volt);
  blockTrack3.spin(forward, 12, volt);
  blockTrack4.spin(forward, 12, volt);
  blockTrack5.stop(brake);
  drive(1400, 1);
  //After picking up first three balls:
  turn(370, 0);
  drive(760, 1);
  blockTrack3.spin(reverse,12, volt);
  blockTrack2.spin(reverse,12, volt);
  blockTrack4.spin(reverse, 12, volt);
  blockTrack5.spin(reverse, 4, volt);
  blockTrack1.spin(reverse,5, volt);
  wait(3, sec);//3 seconds for full unload, 1.22 for one
  blockTrack1.stop();
  blockTrack2.stop();
  blockTrack3.stop();
  blockTrack4.stop();
  blockTrack5.stop();
  blockTrack1.spin(forward,12, volt);
  //After attempting to score first ball:
  blockTrack1.spin(forward, 12, volt);
  blockTrack3.spin(forward, 12, volt);
  blockTrack2.spin(forward, 12, volt);
  blockTrack4.spin(forward, 12, volt);
  setVelocity(40);
  drive(2100, 0);
  setVelocity(20);
  turn(450, 0);
  drive(600, 0);
  setVelocity(20);
  turn(370, 0); 
  leftDrive.setTimeout(1, sec);
  rightDrive.setTimeout(1,sec);
  setVelocity(65);
  drive(1300, 0);
  //After lining up with goal:
  unloader.set(true);
  blockTrack1.spin(forward, 12, volt);
  blockTrack2.spin(forward, 12, volt);
  blockTrack3.spin(forward, 12, volt);
  blockTrack4.spin(forward, 12, volt);
  blockTrack5.stop(brake);
  setVelocity(50);
  drive(1600, 1);
  setVelocity(30);
  leftDrive.setTimeout(0.4, sec);
  rightDrive.setTimeout(0.4,sec);
  drive(400,1);
  wait(0.5, sec);
  setVelocity(40);
  drive(600, 0);
  /*
  if(leftWallDist.objectDistance(mm) <= 340 && leftWallDist.objectDistance(mm) >= 290){
    turn(45, 0);
  }
  else if(leftWallDist.objectDistance(mm) >= 410 && leftWallDist.objectDistance(mm) <= 460){
    turn(45, 1);
  }
  else if(leftWallDist.objectDistance(mm) <= 320 && leftWallDist.objectDistance(mm) >= 270){
    turn(90, 0);
  }
  else if(leftWallDist.objectDistance(mm) >= 460 && leftWallDist.objectDistance(mm) <= 510){
    turn(90, 1);
  }
  leftDrive.setTimeout(0.5, sec);
  rightDrive.setTimeout(0.5,sec);
  drive(900, 0);
  blockTrack.spin(forward, 12, volt);
 }
*/
}

void usercontrol(void) {
  
  while (1) {
    // printf("AutonSelected: %d\n", currentAuton);
    if(!calibrated){
      int startTime = Brain.Timer.time(msec);
      InertialSensor.calibrate();
      while(InertialSensor.isCalibrating()){
        printf("Calibrating...\n");
        wait(100, msec);
      }
      int calibrationTime = Brain.Timer.time(msec) - startTime;
      printf("Calibrated in %dms\n", calibrationTime);
      calibrated = true;
    }



   // updateDriveSpeed();
    updateRobotPosition();  
    brainUI();

    if(Controller.ButtonLeft.pressing()){
leftDrive.spin(reverse, turnPID(90), volt);
rightDrive.spin(forward, turnPID(90), volt);
    }
    else{
     updateDriveSpeed();
    }

   // leftDrive.spin(forward, -Controller.Axis3.position() * 0.12 - turnPID(Controller.Axis1.position()*1.8), volt);
   // rightDrive.spin(forward, -Controller.Axis3.position() * 0.12 + turnPID(Controller.Axis1.position()*1.8), volt);

    if(Controller.ButtonUp.PRESSED){
    setVelocity(60);  
    turn(45, 0);}
    if(Controller.ButtonDown.PRESSED){
    setVelocity(60);  
    turn(45, 1);}
    if(Controller.ButtonRight.PRESSED){
      setVelocity(10);
      drive(1000, 1);
      drive(1000, 0);

    }
    

    if(Controller.ButtonR2.pressing()){
      lowBlockTrack.spin(forward, 12, volt);
      highBlockTrack.spin(forward, 12, volt); 
    } else if(Controller.ButtonR1.pressing()){
        lowBlockTrack.spin(reverse, 12, volt);
        highBlockTrack.spin(reverse, 12, volt);
    } else if(Controller.ButtonL2.pressing()){
      lowBlockTrack.spin(forward, 12, volt);
      highBlockTrack.spin(reverse, 12, volt);
    } else if(Controller.ButtonL1.pressing()){
      lowBlockTrack.spin(forward, 12, volt);
      highBlockTrack.setStopping(brake);
    } else {
      lowBlockTrack.stop();
      highBlockTrack.stop();
    }

    if(Controller.ButtonX.PRESSED || Controller.ButtonA.PRESSED){
      unloader.set(!unloader.value());
    }
    if(Controller.ButtonY.PRESSED || Controller.ButtonB.PRESSED){
      descore.set(!descore.value());
    }

    wait(10, msec); // Sleep the task for a short amount of time to prevent wasted resources.
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

