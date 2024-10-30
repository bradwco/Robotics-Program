#ifndef PID_CONTROLLER_H //if not defined
#define PID_CONTROLLER_H    
#include "vex.h"

class PIDController{
    //Controller Gains (proportional, integral, derivative gains)
    private:
    double Kp;
    double Ki;
    double Kd;
    double integralErrorLim;

    double integralErrorSum;
    double prevError;

    vex::timer time;

    bool pidStart = false;

    public:
    PIDController(double Kp, double Ki, double Kd, double integralErrorLim)
        : Kp(Kp), Ki(Ki), Kd(Kd), integralErrorLim(integralErrorLim){};

    void PIDControllerStart(){
        pidStart = true;
        integralErrorSum = 0;
        prevError = 0;
        time.reset();
    }

    float PIDControllerUpdate(double error){
        if (!pidStart) return 0.0;  // ensures the PIDSTART has been ran
        
        int deltaT = time.time(vex::timeUnits::msec); 
        if (deltaT == 0){
            return Kp*error;
        }

        time.reset();
        double derivative = (error-prevError)/deltaT;

        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) {
            integralErrorSum = 0;
        } else {
            // Sum up the integral error and check limit
            if (integralErrorLim > 0) {
                integralErrorSum +=  error * deltaT;
                if(integralErrorSum < -integralErrorLim){
                    integralErrorSum =  -integralErrorLim;
                }
                else if(integralErrorSum > integralErrorLim){
                    integralErrorSum = integralErrorLim;
                }
            }

        }

        prevError = error;
        //printf("info %f,  %f,  %f \n", (Kp * error), (integralErrorSum * Ki), (Kd * derivative));
        return (Kp * error)  + (integralErrorSum * Ki) + (Kd * derivative);
        }
    
};

#endif