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

inertial InertialSensor = inertial(PORT18);

motor topLeftDrive = motor(PORT4, ratio6_1, true);
motor bottomLeftDrive = motor(PORT3, ratio6_1, false);
motor_group leftDrive = motor_group(topLeftDrive, bottomLeftDrive);

motor topRightDrive = motor(PORT2, ratio6_1, false);
motor bottomRightDrive = motor(PORT1, ratio6_1, true);
motor_group rightDrive = motor_group(topRightDrive, bottomRightDrive);

motor blockTrack1 = motor(PORT11, gearSetting::ratio18_1, true);
motor blockTrack2 = motor(PORT12, gearSetting::ratio18_1, true);
motor blockTrack3 = motor(PORT13, gearSetting::ratio18_1, true);
motor blockTrack4 = motor(PORT14, gearSetting::ratio18_1, true);
motor blockTrack5 = motor(PORT15, gearSetting::ratio18_1, true);
motor_group blockTrack = motor_group(blockTrack1, blockTrack2, blockTrack3, blockTrack4, blockTrack5);

//Radio in 21

digital_out unloader = digital_out(Brain.ThreeWirePort.A);


int screen = 0;

int motorStatusTimer = 0;
bool frontLeftStatus = topLeftDrive.installed();
bool backLeftStatus = bottomLeftDrive.installed();
bool frontRightStatus = topRightDrive.installed();
bool backRightStatus = bottomRightDrive.installed();
bool blockTrack1Status = blockTrack1.installed();
bool blockTrack2Status = blockTrack2.installed();
bool blockTrack3Status = blockTrack3.installed();
bool blockTrack4Status = blockTrack4.installed();
bool blockTrack5Status = blockTrack5.installed();
bool distanceStatus = topBlockDist.installed();
bool distance2Status = midBlockDist.installed();

bool calibrated = false;

double inchesPerTick = 0.0279;

double robotX = 0.0;
double robotY = 0.0;
double robotHeading = 0.0;
double prevLeftTicks = 0.0;
double prevRightTicks = 0.0;

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

void updateDriveSpeed(void){

  double forwardVal = -Controller.Axis3.position();
  double turnVal = -Controller.Axis1.position();
  double leftPower = forwardVal + turnVal;
  double rightPower = forwardVal - turnVal;

  double leftVoltage = leftPower * 0.12;
  double rightVoltage = rightPower * 0.12;

  topRightDrive.spin(vex::forward, rightVoltage, vex::voltageUnits::volt);
  bottomRightDrive.spin(vex::forward, rightVoltage, vex::voltageUnits::volt);
  topLeftDrive.spin(vex::forward, leftVoltage, vex::voltageUnits::volt);
  bottomLeftDrive.spin(vex::forward, leftVoltage, vex::voltageUnits::volt);

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

void driveForward(int degreeNum) {

    leftDrive.spinFor(reverse, degreeNum, degrees, false);
    rightDrive.spinFor(degreeNum, degrees);

  }
void setVelocity(int velocity) {

    leftDrive.setVelocity(velocity, percent);
    rightDrive.setVelocity(velocity, percent);

  }

void turnPID(int targetAngle, int tolerance) {

  double kP = 0.3;  
  double kI = 0.008;
  double kD = 2.6; 
  double integral = 0;
  double previousError = 0;
  double currentAngle = InertialSensor.heading();
  double error = targetAngle - currentAngle;

  while (abs(error) > tolerance) {
    currentAngle = InertialSensor.heading();
    error = targetAngle - currentAngle;
    
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    if (abs(error) < 20) {
      integral += error;
    } else {
      integral = 0;
    }
    
    if (integral > 50) integral = 50;
    if (integral < -50) integral = -50;
    
    double derivative = error - previousError;
    
    double output = (kP * error) + (kI * integral) + (kD * derivative);
    
    if (abs(output) > 0 && abs(output) < 2.0) {
      if (output > 0) output = 2.0;
      if (output < 0) output = -2.0;
    }
    
    if (output > 12) output = 12;
    if (output < -12) output = -12;
    
    leftDrive.spin(reverse, output, volt);
    rightDrive.spin(forward, -output, volt);
    
    previousError = error;
    wait(10, msec);
  }

  leftDrive.stop(brake);
  rightDrive.stop(brake);

}

void driveTicks(double inches){
  double ticks = inches / inchesPerTick;
  leftDrive.spinFor(reverse, ticks, vex::rotationUnits::raw, false);
  rightDrive.spinFor(ticks, vex::rotationUnits::raw);
}

void driveReverse(int degreeNum) {

    leftDrive.spinFor(forward, degreeNum, degrees, false);
    rightDrive.spinFor(reverse, degreeNum, degrees);

  }

void driveToPoint(int x, int y) {
  double deltaX = x - robotX;
  double deltaY = y - robotY;
  
  double distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  double targetAngle = atan2(deltaX, deltaY) * 180.0 / M_PI;
  
  if (targetAngle < 0) {
    targetAngle += 360;
  } 
  
  turnPID(targetAngle, 1);

  setVelocity(80);
  driveTicks(distance);

  robotX = x;
  robotY = y;
  robotHeading = targetAngle;
}

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

void loadBalls(){
  while (loadingBlocks) {
    blockTrack1.spin(forward, 12, volt);
    if(midBlockDist.objectDistance(mm) >= 150){
      blockTrack2.spin(forward, 12, volt);
    } else if(midBlockDist.objectDistance(mm) < 150 && topBlockDist.objectDistance(mm) < 150){
      blockTrack2.stop(hold);
    }
    if(topBlockDist.objectDistance(mm) >= 120){
      blockTrack3.spin(forward, 12, volt);
    } else if(topBlockDist.objectDistance(mm) < 120){
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
    frontLeftStatus = topLeftDrive.installed();
    backLeftStatus = bottomLeftDrive.installed();
    frontRightStatus = topRightDrive.installed();
    backRightStatus = bottomRightDrive.installed();
    blockTrack1Status = blockTrack1.installed();
    blockTrack2Status = blockTrack2.installed();
    blockTrack3Status = blockTrack3.installed();
    blockTrack4Status = blockTrack4.installed();
    blockTrack5Status = blockTrack5.installed();
    distanceStatus = topBlockDist.installed();
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

      drawButton(5 , 212, 60, 20, "Back", 0);
      break;

    case 2:
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);


      Brain.Screen.setPenColor(frontLeftStatus ? white : red);
      Brain.Screen.printAt(10, 20, false, "Top Left Drive (Port 4): %s", frontLeftStatus ? "Connected" : "Disconnected");
      if (frontLeftStatus) {
        double flTemp = topLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(flTemp > 50 ? (flTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 20, false, "%.1f°C", flTemp);
      }

      Brain.Screen.setPenColor(backLeftStatus ? white : red);
      Brain.Screen.printAt(10, 40, false, "Bottom Left Drive (Port 3): %s", backLeftStatus ? "Connected" : "Disconnected");
      if (backLeftStatus) {
        double blTemp = bottomLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(blTemp > 50 ? (blTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 40, false, "%.1f°C", blTemp);
      }

      Brain.Screen.setPenColor(frontRightStatus ? white : red);
      Brain.Screen.printAt(10, 60, false, "Top Right Drive (Port 2): %s", frontRightStatus ? "Connected" : "Disconnected");
      if (frontRightStatus) {
        double frTemp = topRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(frTemp > 50 ? (frTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 60, false, "%.1f°C", frTemp);
      }

      Brain.Screen.setPenColor(backRightStatus ? white : red);
      Brain.Screen.printAt(10, 80, false, "Bottom Right Drive (Port 1): %s", backRightStatus ? "Connected" : "Disconnected");
      if (backRightStatus) {
        double brTemp = bottomRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(brTemp > 50 ? (brTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 80, false, "%.1f°C", brTemp);
      }

      Brain.Screen.setPenColor(blockTrack1Status ? white : red);
      Brain.Screen.printAt(10, 100, false, "Block Track 1 (Port 11): %s", blockTrack1Status ? "Connected" : "Disconnected");
      if (blockTrack1Status) {
        double bt1Temp = blockTrack1.temperature(celsius);
        Brain.Screen.setPenColor(bt1Temp > 50 ? (bt1Temp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 100, false, "%.1f°C", bt1Temp);
      }

      Brain.Screen.setPenColor(blockTrack2Status ? white : red);
      Brain.Screen.printAt(10, 120, false, "Block Track 2 (Port 12): %s", blockTrack2Status ? "Connected" : "Disconnected");
      if (blockTrack2Status) {
        double bt2Temp = blockTrack2.temperature(celsius);
        Brain.Screen.setPenColor(bt2Temp > 50 ? (bt2Temp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 120, false, "%.1f°C", bt2Temp);
      }

      Brain.Screen.setPenColor(blockTrack3Status ? white : red);
      Brain.Screen.printAt(10, 140, false, "Block Track 3 (Port 13): %s", blockTrack3Status ? "Connected" : "Disconnected");
      if (blockTrack3Status) {
        double bt3Temp = blockTrack3.temperature(celsius);
        Brain.Screen.setPenColor(bt3Temp > 50 ? (bt3Temp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 140, false, "%.1f°C", bt3Temp);
      }

      Brain.Screen.setPenColor(blockTrack4Status ? white : red);
      Brain.Screen.printAt(10, 160, false, "Block Track 4 (Port 14): %s", blockTrack4Status ? "Connected" : "Disconnected");
      if (blockTrack4Status) {
        double bt4Temp = blockTrack4.temperature(celsius);
        Brain.Screen.setPenColor(bt4Temp > 50 ? (bt4Temp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 160, false, "%.1f°C", bt4Temp);
      }

      Brain.Screen.setPenColor(blockTrack5Status ? white : red);
      Brain.Screen.printAt(10, 180, false, "Block Track 5 (Port 15): %s", blockTrack5Status ? "Connected" : "Disconnected");
      if (blockTrack5Status) {
        double bt5Temp = blockTrack5.temperature(celsius);
        Brain.Screen.setPenColor(bt5Temp > 50 ? (bt5Temp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 180, false, "%.1f°C", bt5Temp);
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
   if(!calibrated){

    InertialSensor.calibrate();
    while(InertialSensor.isCalibrating()){
            printf("Calibrating...\n");
            wait(100, msec);
          }
    printf("Calibrated\n");
    calibrated = true;
  }
}
void autonomous(void) {
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
  */
 setVelocity(40);
 driveTicks(5);
}

void usercontrol(void) {
  while (1) {
    
    if(!calibrated){

    InertialSensor.calibrate();
    while(InertialSensor.isCalibrating()){
            printf("Calibrating...\n");
            wait(100, msec);
          }
    printf("Calibrated\n");
    calibrated = true;
  }

    updateDriveSpeed();
    updateRobotPosition();  // Update robot X, Y position based on encoders and heading

    
    brainUI();
    
       if(Controller.ButtonRight.PRESSED){
     // turnPID(180, 1);
    }
    if(Controller.ButtonLeft.PRESSED){
      //printf("%.2f\n", InertialSensor.heading());
    }
   
   if(Controller.ButtonUp.pressing()){
   // drivePath(square, 4);
   }
    if(Controller.ButtonR2.pressing()){
     blockTrack.spin(forward, 12, volt);
    }
    else if(Controller.ButtonR1.pressing()){
     blockTrack.spin(reverse, 12, volt);
    }
    else if(Controller.ButtonL1.pressing()){
     blockTrack1.spin(forward, 12, volt);
      blockTrack2.spin(forward, 12, volt);
      blockTrack3.spin(forward, 12, volt);
      blockTrack4.spin(reverse, 12, volt);
      blockTrack5.spin(reverse, 12, volt);
    }
    else{
     blockTrack.stop();
    }
      if(Controller.ButtonX.PRESSED || Controller.ButtonA.PRESSED){

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
