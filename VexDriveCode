#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS 


// Robot configuration code. 
motor leftMotorA = motor(PORT2, ratio18_1, false);
motor leftMotorB = motor(PORT3, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT6, ratio18_1, true);
motor rightMotorB = motor(PORT7, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 294.89399999999995, 39.878, mm, 1);

motor ChainMotorA = motor(PORT19, ratio18_1, false);
motor ChainMotorB = motor(PORT20, ratio18_1, true);
motor_group Chain = motor_group(ChainMotorA, ChainMotorB);

motor Intake = motor(PORT10, ratio36_1, false);

digital_out claw = digital_out(Brain.ThreeWirePort.A);
controller Controller1 = controller(primary);


// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {Allen Liu}                                               */
/*    Created:      {10/19/26}                                                */
/*    Description:  Read ReadMe                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;

//This is the length of the robot and creation of the current degrees and x/y position of the robot
double currentX = 72; 
double currentY = 8; //16 start //19 after
double currentD = 0;
double tempCurrentD=0;

//calcX calcX creates the x vector(Position) based on the angle;
double calcX(double degree){
  degree *= 180;
  degree = degree / M_PI;
  return 9.5*sin(degree);
}
 
//calcY calcY creates the y vector based on the angles
double calcY(double degree){
  degree =  9.5*cos(degree);
  degree *= 180;
  return degree / M_PI;
}

//move the robot and pause code until it is done
void move(double x){
  Drivetrain.driveFor(forward, x, inches,true);
}

void trn(double x){
  Drivetrain.turnFor(left, x,degrees,false);
} 

void turnTo(double x,double ms){
  double degree = currentD - x;
  if(fabs(degree)-180>0){
    if(degree < 0){
      degree+=180;
    }
    else {  
      degree-=180;
    } 
  }
  trn(degree);
  while(ms != 0|| Drivetrain.isMoving()){
    wait(1/1000, seconds);
    ms--; 
  }
} 
 
void cordMove(double x, double y, double ms, bool r){
  double a = currentX-x;
  double b = currentY-y;
  double c = sqrt(pow(fabs(a),2)+pow(fabs(b),2));
  double d = atan(b/a);
  d = fabs(d * 180 / M_PI);
  if(a<0){
    //either cord II or cord III
    if(b<0){
      //cord III
      a = a-calcX(d);
      b = b-calcY(d);
      d += 180;
    }
    else{
      a = a-calcX(d);
      b = b+calcY(d);
      d += 90;
    }
  }
  else{
    //either cord I or IV
    if(b<0){
      //cord IV
      a = a + calcX(d);
      b = b - calcY(d);
      d+=270;
    }
    else{
      a = a + calcX(d);
      b = b - calcY(d);
    }
  }
  if(d-currentD>180){
    d = d-currentD-180;
  }
  else{
    d -= currentD;
  }
  currentD = d;
  currentX = a;
  currentY = b;
  c = sqrt(pow(fabs(a),2)+pow(fabs(b),2));

  if(r){
    trn(d*-1);
    currentD = d * -1;
    move(c*-1);
  }
  else{
    trn(d);
    move(c);
  }
  
  while(ms != 0|| Drivetrain.isMoving()){
    wait(1/1000, seconds);
    ms--; 
  }
} 

void intakeStart(){
  Intake.spin(forward);
}
 
void intakeStop(){ 
  Intake.stop(); 
} 

int chainHeigth = 0;
int chainCurrent = 0;
void ChainNextCup(){
  chainHeigth+=1;
  Chain.spinFor(forward,1,turns); 
} 

void ChainReturn(){
  Chain.spinFor(forward,chainHeigth,turns);
} 

void ChainBottom(){
  Chain.spinFor(reverse,chainHeigth,turns);
} 

void ChainNextCup(int x){
  chainHeigth+=1;
  Chain.spinFor(forward,1,turns,false);
} 

void ChainReturn(int x){
  Chain.spinFor(forward,chainHeigth,turns,false);
} 

void ChainBottom(int x){
  Chain.spinFor(reverse,chainHeigth,turns,false); 
} 

void AtonOption1(){
  //flip the switch twice
  cordMove(72,16,2000,true);
  cordMove(72,9.6,0,false);
  cordMove(72,16,2000,true);
  cordMove(72,9.6,0,false);
  cordMove(72,16,2000,true);

  ChainNextCup(); 
  cordMove(48,24,2000,true);
  claw.set(false);

  intakeStart();
  cordMove(0,12,2000,false);
  claw.set(true);
  intakeStop();
  ChainReturn(1);
  cordMove(48,24,2000,false); 
  ChainNextCup();
  claw.set(false);
  ChainBottom();
  chainHeigth = 0;
} 
  
void AtonOption2(){ 
  //flip spinners
  Drivetrain.setDriveVelocity(100,percent);
  claw.set(true);
  cordMove(72,16,2000,true);
  cordMove(72,0,2000,false);
  cordMove(72,16,2000,true);

  //input help cup 
  ChainNextCup(1);
  cordMove(48,24,2000,true);
  claw.set(false);
  ChainBottom(1);
  
  //input side yellow cup 
  intakeStart();
  cordMove(48,0,2000,false);
  claw.set(true);
  ChainReturn(1);
  cordMove(48,0,2000,true);
  ChainNextCup();
  claw.set(false);
  intakeStop();
 
  //move to better ending position
  cordMove(24,12,2000,false);
  turnTo(180,0); 
}  
 

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE! 
  vexcodeInit();
  // Begin project code
  

  //move to the input a pin
  while(true){
    if(Controller1.ButtonUp.pressing()){
      AtonOption1();
    }
    else{
      if(Controller1.ButtonX.pressing()){
        ChainNextCup(1);
      }   
      else if(Controller1.ButtonA.pressing()){
        ChainReturn(1);
      }
      else if(Controller1.ButtonB.pressing()){
        ChainBottom(1);
      }
      else if(Controller1.ButtonY.pressing()){
        chainHeigth = 0;
      }

      if(Controller1.Axis3.position()!=0){
        if(Controller1.Axis3.position()>0){
          Drivetrain.setDriveVelocity(Controller1.Axis3.position(),percent);
          Drivetrain.drive(forward);
        }
        else{
          Drivetrain.setDriveVelocity(abs(Controller1.Axis3.position()),percent);
          Drivetrain.drive(reverse);
        }
      }
      if(Controller1.Axis4.position()!= 0){
       if(Controller1.Axis4.position()>0){
          Drivetrain.setTurnVelocity(Controller1.Axis4.position(),percent);
          Drivetrain.turn(right);
        }
        else{
          Drivetrain.setTurnVelocity(abs(Controller1.Axis4.position()),percent);
          Drivetrain.turn(left);
        }
      }
      if(Controller1.ButtonL1.pressing()){
        claw.set(true); 
      }
      if(Controller1.ButtonL2.pressing()){
        claw.set(false);
      }
      Drivetrain.stop();
    }
  } 
}

