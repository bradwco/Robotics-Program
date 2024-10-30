#ifndef AUTONDRIVETRAIN
#define AUTONDRIVETRAIN
#include "pid.h"
#include "vex.h"
#include <iostream>
#include <math.h>

using namespace vex;    

//wheel size
int largeWheel = 320; //4" wheels

//LPCController -> Controls when to "give up"
class LPCController{
    public:
    LPCController(double tolerance, double turnTolerance)
    : tolerance(tolerance), turnTolerance(turnTolerance){}
    
    bool isInTolerance(double error){
        return fabs(error) <= tolerance;
    }
    bool isInTurnTolerance(double error){
        return fabs(error) <= turnTolerance;
    }

    private:
    double tolerance;
    double turnTolerance;
};

class AutonDriveTrain{
public:
    //constructor for drive train
    AutonDriveTrain(motor_group& motorGroupL, motor_group& motorGroupR, LPCController& lpc, inertial& inertialSensor, int timeout)
    : motorGroupL(motorGroupL), motorGroupR(motorGroupR), lpc(lpc), timeout(timeout), wheelCircum(largeWheel), iSensor(inertialSensor) {
        MatchTimer.reset();
    }

    //Set the PID of each side of the drivetrain to the specified values
    void setPID(PIDController *PIDArr){
        leftPID = &PIDArr[0];
        rightPID = &PIDArr[0];
    }
    

   // Parameters: direction type, motor power percentage (0-100), target distance, units, timeout duration (ms)
    bool PIDMove(directionType dir, int mPwr, double distance, distanceUnits units){
    if (dir==reverse){distance=-distance;}

    motorGroupL.resetPosition();
    motorGroupR.resetPosition();

    //Initialize PID
    leftPID->PIDControllerStart();
    rightPID->PIDControllerStart();
    
    while(true){
        //check to see if we went over match time
        if(!checkTimeout()){return false;}
        
        double leftError = distance - revToMM(motorGroupL);
        double rightError = distance - revToMM(motorGroupR);
        //printf("Current Travel L: %f, Current Travel R: %f", motorGroupL.position(turns), motorGroupR.position(turns));

        double leftPIDOutput = leftPID->PIDControllerUpdate(leftError);
        double rightPIDOutput = rightPID->PIDControllerUpdate(rightError);
        //printf("Left Error : %f, Right Error: %f Left Power: %f Right Power: %f\n", leftError ,rightError, leftPIDOutput, rightPIDOutput);

        //scale the output to a certain percent for motor safety (limiting)
        double leftPower = scaleOutput(leftPIDOutput, mPwr);
        double rightPower = scaleOutput(rightPIDOutput, mPwr);   
        
        motorGroupL.spin(forward, leftPower, percent);
        motorGroupR.spin(forward, rightPower, percent);

        //constantly checks to see if we are in tolerance on both sides
        if(lpc.isInTolerance(leftError) && lpc.isInTolerance(rightError)){
            motorGroupL.stop(brake);
            motorGroupR.stop(brake);
            printf("move completed\n");
            return true; 
        }

        //TEMP SLEEP before next loop iteration
        vex::task::sleep(20);
    }
    }

    //target deg, OPTIONAL: clockwise(c) or counterclockwise(cc), if not it will default to best option
    bool PIDTurn(double targetDeg, double maxSpeed = 100, double minSpeed = 1, double kSens = (100/180), std::string cOrCC = ""){
        //north = 0, east = 90, south = 180, west = 270

        double motorPower = 0;
        std::pair<std::string, double> turnData = getTurnData(iSensor.heading(degrees), targetDeg);
        std::string direction = cOrCC;
        if(cOrCC==""){direction=turnData.first;}

            while(true){
                if(!checkTimeout()){return false;} //check match timer

                turnData = getTurnData(iSensor.heading(degrees), targetDeg);
                printf("Heading: %f\n", iSensor.heading(degrees));
                motorPower = getTurnSpeed(turnData.second, maxSpeed, minSpeed, kSens);  //100/180 = 0.556
                if(direction=="c"){
                    motorGroupL.spin(forward, motorPower, percent);
                    motorGroupR.spin(reverse, motorPower, percent);
                    if(lpc.isInTurnTolerance(turnData.second)){ 
                        motorGroupL.stop(brake);
                        motorGroupR.stop(brake);
                        printf("Clockwise Turn Completed\n");
                        return true; 
                    }
                }
                else if(direction=="cc"){
                    motorGroupL.spin(reverse, motorPower, percent);
                    motorGroupR.spin(forward, motorPower, percent);
                    if(lpc.isInTurnTolerance(turnData.second)){ 
                        motorGroupL.stop(brake);
                        motorGroupR.stop(brake);
                        printf("CounterClockwise Turn Completed\n");
                        return true; 
                    }
                }
            }
    }
    

private:
    vex::timer MatchTimer;
    motor_group& motorGroupL;
    motor_group& motorGroupR;

    PIDController* leftPID;
    PIDController* rightPID;
    
    LPCController lpc;

    inertial iSensor;
    //drivetrain parameters
    int maxTurnRate;
    double wheelCircum;
    
    int timeout;

    //convert rotations -> mm
    double revToMM(motor_group& motorGroupT){
        return motorGroupT.position(rev) * wheelCircum;
    }

    //returns direction type and the angular distance
    std::pair<std::string, double>getTurnData(double currentAngle, double targetAngle){
        double clockwise_Diff = 0;
        double counterClockwise_Diff = 0;
        (targetAngle-currentAngle <0)? clockwise_Diff = targetAngle-currentAngle+360: clockwise_Diff = targetAngle-currentAngle;
        (currentAngle-targetAngle <0)? counterClockwise_Diff = currentAngle-targetAngle+360: counterClockwise_Diff = currentAngle-targetAngle;
        if(clockwise_Diff <= counterClockwise_Diff){
            return std::make_pair("c",clockwise_Diff);
        }
        else{
            return std::make_pair("cc",counterClockwise_Diff);
        }
    }

    //kSens controls the sensitivity of how intense the robot will decrease speed as approaching target
    double getTurnSpeed(double angular_distance, double max_speed, double min_speed, double kSens){
        return std::max(min_speed, std::min(max_speed, kSens*angular_distance));
    }

    double scaleOutput(double output, int maxPower) {
        if (output > maxPower) return maxPower;
        if (output < -maxPower) return -maxPower;
        return output;
    }
    bool checkTimeout(){
        if(MatchTimer.time(msec) > timeout){
            motorGroupL.stop(brake);
            motorGroupR.stop(brake);
            printf("match timer exceeded\n");
            return false;
        }
        return true;
    }
};



#endif