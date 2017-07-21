/*
 * Name: aclmccontroller.h
 *
 * Written by Rakshit Allamraju
 * Date Created 19 July 2017
 *
 * Control algorithm designed by Mark Culter from MIT ACL labs
 */

#pragma once

#include <stdio.h>
#include <math.h>
#include "../globals/global.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "../../include/imu/sensorfnc.h"

class ACLMCController{
private:
    ACLMCController();
    ~ACLMCController();

    double mass, dT;
    Eigen::Matrix4f ThrustMap;
    Eigen::Matrix3f kp, kd, Kp, Kd;
    Eigen::Vector3f r, r_dot, r_ddot;
    Eigen::Vector3f desiredr, desiredr_dot, desiredr_ddot, desiredr_dddot;
    Eigen::Vector3f desiredr_prev, desiredr_dot_prev, desiredr_ddot_prev;
    float psi_d, psi_d_prev;
    Eigen::Vector3f r_fb;
    Eigen::Vector3f e, e_dot;
    Eigen::Vector3f Omega;

    float f_total;
    Eigen::Vector3f M_b;

    Eigen::Quaternionf q_imu, q_d, q_e;

    Eigen::Vector3f Fbar_i, Fbar_b, Fdot_i, F_i_prev;
    Eigen::Vector3f gravity_vec;

public:
    void PrintDetails();
    void setPosKp(Eigen::Matrix3f);
    void setPosKd(Eigen::Matrix3f);
    void setAttKp(Eigen::Matrix3f);
    void setAttKd(Eigen::Matrix3f);
    void setMap(float armlength, float dragcoeff);
    void loadDesired(float r_d[3], float psi_d);
    void loadGPSIMU(float GPS[3], float IMU[3], float W[3], home myHome);
    void getThrusts(float f_total, Eigen::Vector3f);
    void GenerateControl();

};
