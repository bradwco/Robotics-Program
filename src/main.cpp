/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Bradley                                                   */
/*    Created:      9/20/2024, 11:15:14 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include "vex.h"
#include "pid.h"
#include "AutonProgram.h"

using namespace vex;

competition Competition;

brain Brain;
controller Controller1;

gearSetting stdGear = ratio18_1;
motor motorL1(PORT11, stdGear, false);
motor motorL2(PORT12, stdGear, true);
motor motorL3(PORT13, stdGear, false);
motor motorR1(PORT1, stdGear, true);
motor motorR2(PORT2, stdGear, false);
motor motorR3(PORT3, stdGear, true);

digital_out solenoid(Brain.ThreeWirePort.H);

inertial inertialSensor(PORT10);

motor_group motorGroupL(motorL1, motorL2, motorL3);
motor_group motorGroupR(motorR1, motorR2, motorR3);


//PIDs
//                      Move PID    
//                      {Kp, Ki, Kd, integralErrorLimit}
PIDController PID[1] = {{1, 0, 10, 0}};
//Create an instance of LPC Controller to handle the tolerance
//                               tolerance, turnTolerance
LPCController lpc = LPCController(1.5, 0.6); 

void pre_auton(void) {
  inertialSensor.startCalibration();// NOTE: MOVE THIS LINE AND THE ONE BELOW TO PRE-AUTON
  printf("-------NEW TEST-------\n");
  printf("INERTIAL SENSOR HAS CALIBRATED\n");
  printf("STARTING Heading: %f\n", inertialSensor.heading(degrees));
  vex::task::sleep(4000); // ALSO retrieve currentDeg once while in the preauton phase
}


void autonomous(void) { 
  AutonDriveTrain autonDriveTrain(motorGroupL, motorGroupR, lpc, inertialSensor, 60000); //timeout limit for FULL run (ms)
  

  autonDriveTrain.setPID(PID);
  //PIDMOVE (direction, mPwr, dist, units)
  //1 Tile(2ft) = 610mm
    if (autonDriveTrain.PIDMove(forward, 20, 610, distanceUnits::mm)) {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print ("MOVE 1 FINISHED");
    }
    if (autonDriveTrain.PIDMove(reverse, 20, 305, distanceUnits::mm)) {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print ("MOVE 2 FINISHED");
    }
    //PARAM: (target deg, maxSpeed, minSpeed, kSens, OPTIONAL: c or CC,)
    //kSens controls the sensitivity of how intense the robot will decrease speed as approaching target
    if (autonDriveTrain.PIDTurn(270, 80, 10, 0.3)){
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print ("TURN FINISHED");
        vex::task::sleep(1000);  
    }
    if (autonDriveTrain.PIDTurn(0, 80, 10, 0.3)){
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print ("TURN FINISHED");
        vex::task::sleep(1000);  
    }
    if (autonDriveTrain.PIDTurn(90, 80, 10, 0.3)){
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print ("TURN FINISHED");
        vex::task::sleep(1000);  
    }
}


void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print ("Now running DRIVER PROGRAM");
    if(Controller1.ButtonX.pressing()) {
        Controller1.rumble("...");
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print ("Now running AUTON1 PROGRAM");
        wait(1500, msec);
        autonomous();
    }
  vex::wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    vex::wait(100, msec);
  }
}
