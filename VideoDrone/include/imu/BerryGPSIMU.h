/* FileName: BerryGPSIMU.h
Created by Rakshit Allamraju
Date 2 July 2017

IMU Sensor thread
*/

#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include "LSM9DS0.h"
#include <linux/i2c-dev.h>
#include "../../include/globals/globalstruct.h"

float kalmanFilterX(float accAngle, float gyroRate);
float kalmanFilterY(float accAngle, float gyroRate);

void* readGPSIMU();

