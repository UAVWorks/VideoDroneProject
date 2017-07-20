/*
 * Name: aclmccontroller.h
 *
 * Written by Rakshit Allamraju
 * Date Created 19 July 2017
 *
 * Control algorithm designed by Mark Culter from MIT ACL labs
 */

#ifndef ACLMCCONTROLLER_H
#define ACLMCCONTROLLER_H

#include <stdio.h>
#include <math.h>
#include "armadillo"
#include <boost/math/quaternion.hpp>

class ACLMCController{
private:
    ACLMCController();
    ~ACLMCController();

    double mass, dT;
    arma::colvec r, r_dot, r_ddot;
    arma::colvec desiredr, desiredr_dot, desiredr_ddot;
    arma::colvec desiredr_prev, desiredr_dot_prev, desiredr_ddot_prev;
    float psi_d;
    arma::colvec r_fb;
    arma::colvec e, e_dot;

    float f_total;
    arma::colvec M_b;

    boost::math::quaternion<float> q_imu, q_d, q_e;
    arma::colvec Fbar_i, Fbar_b;
    arma::colvec gravity_vec;

public:
    void PrintDetails();
    void loadDesired(float r_d[3], float psi_d);
    void loadGPSIMU(float GPS[3], float IMU[3]);
    void GenerateControl();

};

#endif // ACLMCCONTROLLER_H
