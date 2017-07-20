/* FileName: PID.cpp
Created by Rakshit Allamraju
Date 30 June 2017

Contains class definitions for various controller to be used
in this project

*/

#include "../../include/control/PID.h"

PID::PID(){
    dT = 0.0;

    Kp = 0.0;
    Ki = 0.0;
    Kd = 0.0;

    Yd = 0.0;
    Ya = 0.0;

    error = 0.0;
    errorDerivative = 0.0;
    errorIntegral = 0.0;

    error_prev = 0.0;
}

PID::~PID(){

}

void PID::setDt(float dt){
    dT = dt;
}

void PID::setActual(float ya){
    Ya = ya;
}

void PID::setDesired(float yd){
    Yd = yd;
}

void PID::setKp(float kp){
    Kp = kp;
}

void PID::setKd(float kd){
    Kd = kd;
}

void PID::setKi(float ki){
    Ki = ki;
}

void PID::calcError(){
    error = Yd-Ya;
}

void PID::calcIntegralError(){
    errorIntegral += errorIntegral*dT;
}

void PID::calcDerivativeError(){
    errorDerivative = (error - error_prev)*(1.0/dT);
    error_prev = error;
}

void PID::calcPID(){
    calcError();
    calcIntegralError();
    calcDerivativeError();

    output = Kp*error + Ki*errorIntegral + Kd*errorDerivative;
}

float PID::getoutput(){
    return output;
}

