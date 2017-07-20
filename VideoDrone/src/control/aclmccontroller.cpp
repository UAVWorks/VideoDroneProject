/*
 * Name: aclmccontroller.cpp
 *
 * Written by Rakshit Allamraju
 * Date Created 19 July 2017
 *
 * Control algorithm designed by Mark Culter from MIT ACL labs
 */

#include "../../include/control/aclmccontroller.h"


ACLMCController::ACLMCController(){

    mass = 10;//TEMP ,load from config
    dT = 1.0/10.0;

    r.set_size(3);
    r.fill(0.0);
    r_dot.set_size(3);
    r_dot.fill(0.0);
    r_ddot.set_size(3);
    r_dot.fill(0.0);

    r_fb.set_size(3);
    r_fb.fill(0.0);

    desiredr.set_size(3);
    desiredr_dot.set_size(3);
    desiredr_ddot.set_size(3);
    desiredr.fill(0.0);
    desiredr_dot.fill(0.0);
    desiredr_ddot.fill(0.0);

    desiredr_prev.set_size(3);
    desiredr_dot_prev.set_size(3);
    desiredr_ddot_prev.set_size(3);
    desiredr_prev.fill(0.0);
    desiredr_dot_prev.fill(0.0);
    desiredr_ddot_prev.fill(0.0);

    e.set_size(3);
    e.fill(0.0);

    e_dot.set_size(3);
    e_dot.fill(0.0);

    q_imu.R_component_1() = 0.0;
    q_imu.R_component_2() = 0.0;
    q_imu.R_component_3() = 0.0;
    q_imu.R_component_4() = 0.0;

    q_d.R_component_1() = 0.0;
    q_d.R_component_2() = 0.0;
    q_d.R_component_3() = 0.0;
    q_d.R_component_4() = 0.0;

    q_e.R_component_1() = 0.0;
    q_e.R_component_2() = 0.0;
    q_e.R_component_3() = 0.0;
    q_e.R_component_4() = 0.0;

    Fbar_b.set_size(3);
    Fbar_b.fill(0.0);

    Fbar_i.set_size(3);
    Fbar_i.fill(0.0);

    gravity_vec(0) = 0;
    gravity_vec(1) = 0;
    gravity_vec(2) = 9.81;

}

void ACLMCController::loadGPSIMU(float GPS[3], float IMU[3]){
    // Convert IMU data to quaternion
    EulertoQuaternion(IMU[0], IMU[1], IMU[2], q_imu);
    gps2ned(GPS, myHome, r);

}

void ACLMCController::loadDesired(float desired_loc[3], float desired_psi){
    desiredr = desired_loc;
    psi_d = desired_psi;
}

void ACLMCController::GenerateControl(){

    arma::colvec F_i(3), F_b(3);
    arma::colvec Fb_test(3);
    Fb_test.fill(0.0);
    F_i.fill(0.0);
    F_b.fill(0.0);

    boost::math::quaternion<float> q_psi_convert(cos(psi_d/2),0,0,sin(psi_d/2));

    desiredr_dot = (desiredr-desiredr_prev)*(1/dT);
    desiredr_prev = desiredr;

    desiredr_ddot = (desiredr_dot - desiredr_dot_prev)*(1/dT);
    desiredr_dot_prev = desiredr_dot;

    e = r - desiredr;
    e_dot = (r_dot - desiredr_dot)*(1/dT);

    r_fb = -dot(Kp,e) - dot(Kd,e_dot); //PD for now

    F_i = (mass*(desiredr_ddot + r_fb + gravity_vec));

    Fb_test(2) = 1;
    if( dot(F_i,Fb_test) >= 0){
        F_b(2) = 1;
    }else{
        F_b(2) = -1;
    }

    Fbar_i = F_i/norm(F_i,2);
    Fbar_b = F_b/norm(F_b,2);

    f_total = norm(F_i,2);

    q_d.R_component_1() = 1+ dot(Fbar_i,Fbar_b);
    q_d.R_component_2() = cross(Fbar_i, Fbar_b)(0);
    q_d.R_component_3() = cross(Fbar_i, Fbar_b)(1);
    q_d.R_component_4() = cross(Fbar_i, Fbar_b)(2);

    q_d *= 1.0/sqrt(2*(1+dot(Fbar_i,Fbar_b)));

    q_d *= q_psi_convert; // desired attitude


    q_e = conj(q_imu)*q_d;

    M_b = -sgn(q_e.R_component_1())*Kp*q_e(1,2,3);
}
