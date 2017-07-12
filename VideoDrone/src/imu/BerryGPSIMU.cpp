/* FileName: BerryGPSIMU.cpp
Created by Rakshit Allamraju
Date 10 July 2017

Thread to run the IMU and GPS data using the BERRYGPSIMU module for the rPI and write to a global variable
*/

#include "../../include/control/BerryGPSIMU.h"
#include "../../include/globals/global.h"

void* readIMU(){

char filename[20];
sprintf(filename, "/dev/i2c-%d", 1);
file = open(filename, O_RDWR);
if (file<0) {
        printf("Unable to open I2C bus!");
        exit(1);
}


writeAccReg(CTRL_REG1_XM, 0b01100111); 
writeAccReg(CTRL_REG2_XM, 0b00100000); 


}

void selectDevice(int file, int addr)
{
 char device[3];
 
 if (ioctl(file, I2C_SLAVE, addr) < 0) {
 printf("Failed to select I2C device.");
 }
}
