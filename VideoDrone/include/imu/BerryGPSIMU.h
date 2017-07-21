/* FileName: BerryGPSIMU.h
Created by Rakshit Allamraju
Date 2 July 2017

IMU Sensor thread
*/

#pragma once

#include "../globals/globalstruct.h"
//#include "sensor.h"

float kalmanFilterX(float accAngle, float gyroRate);
float kalmanFilterY(float accAngle, float gyroRate);

void* readGPSIMU();

