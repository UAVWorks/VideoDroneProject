/* FileName: BerryGPSIMU.cpp
Created by Rakshit Allamraju
Date 10 July 2017

Thread to run the IMU and GPS data using the BERRYGPSIMU module for the rPI and write to a global variable
*/

#include "../../include/imu/BerryGPSIMU.h"
#include "sensor.c"

#define DT 0.02         // [s/loop] loop period. 20ms
#define AA 0.97         // complementary filter constant

#define A_GAIN 0.0573      // [deg/LSB]
#define G_GAIN 0.070     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846

//Used by Kalman Filters
float Q_angle  =  0.01;
float Q_gyro   =  0.0003;
float R_angle  =  0.01;
float x_bias = 0;
float y_bias = 0;
float XP_00 = 0, XP_01 = 0, XP_10 = 0, XP_11 = 0;
float YP_00 = 0, YP_01 = 0, YP_10 = 0, YP_11 = 0;
float KFangleX = 0.0;
float KFangleY = 0.0;



void  INThandler(int sig)
{
        signal(sig, SIG_IGN);
        exit(0);
}

void* readGPSIMU()
{
        bool KALMAN_ON(true);

        // add GPS serial communicationcode
        float rate_gyr_y = 0.0;   // [deg/s]
        float rate_gyr_x = 0.0;    // [deg/s]
        float rate_gyr_z = 0.0;     // [deg/s]

        int  accRaw[3];
        int  magRaw[3];
        int  gyrRaw[3];

        float gyroXangle = 0.0;
        float gyroYangle = 0.0;
        float gyroZangle = 0.0;
        float AccYangle = 0.0;
        float AccXangle = 0.0;
        float CFangleX = 0.0;
        float CFangleY = 0.0;

        signal(SIGINT, INThandler);

        enableIMU();

        //gettimeofday(&tvBegin, NULL);


        while(1)
        {
        //read ACC and GYR data
        readACC(accRaw);
        readGYR(gyrRaw);
        readMAG(magRaw);
        //printf("magRaw X %i    \tmagRaw Y %i \tMagRaw Z %i \n", magRaw[0],magRaw[1],magRaw[2]);

        //Only needed if the heading value does not increase when the magnetometer is rotated clockwise
        magRaw[1] = -magRaw[1];

        //Compute heading
        float heading = 180 * atan2(magRaw[1],magRaw[0])/M_PI;

        //Convert heading to 0 - 360
        if(heading < 0)
        heading += 360;


        //Convert Gyro raw to degrees per second
        rate_gyr_x = (float) gyrRaw[0]  * G_GAIN;
        rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
        rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;


        //Calculate the angles from the gyro
        gyroXangle+=rate_gyr_x*DT;
        gyroYangle+=rate_gyr_y*DT;
        gyroZangle+=rate_gyr_z*DT;


        //Convert Accelerometer values to degrees
        AccXangle = (float) (atan2(accRaw[1],accRaw[2])+M_PI)*RAD_TO_DEG;
        AccYangle = (float) (atan2(accRaw[2],accRaw[0])+M_PI)*RAD_TO_DEG;


        //Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up.
        //Two different pieces of code are used depending on how your IMU is mounted.
        //If IMU is upside down
        /*
        if (AccXangle >180)
                AccXangle -= (float)360.0;

        AccYangle-=90;
        if (AccYangle >180)
                AccYangle -= (float)360.0;
        */

        //If IMU is up the correct way, use these lines
        AccXangle -= (float)180.0;
        if (AccYangle > 90)
                AccYangle -= (float)270;
        else
                AccYangle += (float)90;

        if( KALMAN_ON ){
            //Kalman Filter
            float kalmanX = kalmanFilterX(AccXangle, rate_gyr_x);
            float kalmanY = kalmanFilterY(AccYangle, rate_gyr_y);
            //printf ("\033[22;31mkalmanX %7.3f  \033[22;36mkalmanY %7.3f\t\e[m",kalmanX,kalmanY);

        }

        //Complementary filter used to combine the accelerometer and gyro values.
        CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
        CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;

        //printf("heading %7.3f \t ", heading);

        //printf ("GyroX  %7.3f \t AccXangle \e[m %7.3f \t \033[22;31mCFangleX %7.3f\033[0m\t GyroY  %7.3f \t AccYangle %7.3f \t \033[22;36mCFangleY %7.3f\t\033[0m\n",gyroXangle,AccXangle,CFangleX,gyroYangle,AccYangle,CFangleY);

        }
}


  float kalmanFilterX(float accAngle, float gyroRate)
  {
    float  y, S;
    float K_0, K_1;


    KFangleX += DT * (gyroRate - x_bias);

    XP_00 +=  - DT * (XP_10 + XP_01) + Q_angle * DT;
    XP_01 +=  - DT * XP_11;
    XP_10 +=  - DT * XP_11;
    XP_11 +=  + Q_gyro * DT;

    y = accAngle - KFangleX;
    S = XP_00 + R_angle;
    K_0 = XP_00 / S;
    K_1 = XP_10 / S;

    KFangleX +=  K_0 * y;
    x_bias  +=  K_1 * y;
    XP_00 -= K_0 * XP_00;
    XP_01 -= K_0 * XP_01;
    XP_10 -= K_1 * XP_00;
    XP_11 -= K_1 * XP_01;

    return KFangleX;
  }


  float kalmanFilterY(float accAngle, float gyroRate)
  {
    float  y, S;
    float K_0, K_1;


    KFangleY += DT * (gyroRate - y_bias);

    YP_00 +=  - DT * (YP_10 + YP_01) + Q_angle * DT;
    YP_01 +=  - DT * YP_11;
    YP_10 +=  - DT * YP_11;
    YP_11 +=  + Q_gyro * DT;

    y = accAngle - KFangleY;
    S = YP_00 + R_angle;
    K_0 = YP_00 / S;
    K_1 = YP_10 / S;

    KFangleY +=  K_0 * y;
    y_bias  +=  K_1 * y;
    YP_00 -= K_0 * YP_00;
    YP_01 -= K_0 * YP_01;
    YP_10 -= K_1 * YP_00;
    YP_11 -= K_1 * YP_01;

    return KFangleY;
  }

