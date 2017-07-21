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

    gravity_vec(2) = 9.81;

}

void ACLMCController::setPosKp(Eigen::Matrix3f mat){
    kp = mat;
}

void ACLMCController::setPosKd(Eigen::Matrix3f mat){
    kd = mat;
}

void ACLMCController::setAttKp(Eigen::Matrix3f mat){
    Kp = mat;
}

void ACLMCController::setAttKd(Eigen::Matrix3f mat){
    Kd = mat;
}

void ACLMCController::loadGPSIMU(float GPS[3], float IMU[3], float W[3], home myHome){ // GPS,IMU and Ang Velocity
    // Convert IMU data to quaternion
    EulertoQuaternion(IMU[0], IMU[1], IMU[2], q_imu);
    // get NED data
    gps2ned(GPS, myHome, r);
    Omega(0) = W[0];
    Omega(1) = W[1];
    Omega(2) = W[2];
}

void ACLMCController::loadDesired(float desired_loc[3], float desired_psi){
    desiredr(0) = desired_loc[0];
    desiredr(1) = desired_loc[1];
    desiredr(2) = desired_loc[2];

    psi_d = desired_psi;
}

void ACLMCController::setMap(float d, float c){
    ThrustMap(0,0) = 1;ThrustMap(0,1) = 1;ThrustMap(0,2)=1;ThrustMap(0,3)=1;
    ThrustMap(1,0) = d;ThrustMap(1,1) = 0;ThrustMap(1,2)=-d;ThrustMap(1,3)=0;
    ThrustMap(2,0) = 0;ThrustMap(2,1) = d;ThrustMap(2,2)=0;ThrustMap(2,3)=-d;
    ThrustMap(3,0) = -c;ThrustMap(3,1) = c;ThrustMap(3,2)=-c;ThrustMap(3,3)=c;

}

void ACLMCController::GenerateControl(){

    Eigen::Vector3f F_i, F_b;
    Eigen::Vector3f Fb_test;
    Eigen::Vector3f Fbardot_i, Omega_d;

    float psidot_d;

    Eigen::Quaternionf Q_psid;
    Q_psid.w() = cos(psi_d/2);
    Q_psid.vec()(2) = sin(psi_d/2);

    desiredr_dot = (desiredr-desiredr_prev)*(1/dT);
    desiredr_prev = desiredr;

    desiredr_ddot = (desiredr_dot - desiredr_dot_prev)*(1/dT);
    desiredr_dot_prev = desiredr_dot;

    desiredr_dddot = (desiredr_ddot - desiredr_ddot_prev)*(1/dT);
    desiredr_ddot_prev = desiredr_ddot;

    e = r - desiredr;
    e_dot = (r_dot - desiredr_dot)*(1/dT);

    r_fb = -kp*e - kd*e_dot; //PD for now

    F_i = (mass*(desiredr_ddot + r_fb + gravity_vec));

    Fdot_i = (F_i - F_i_prev)*(1/dT);
    F_i_prev = F_i;

    Fb_test(2) = 1;
    if( F_i.dot(Fb_test) >= 0){
        F_b(2) = 1;
    }else{
        F_b(2) = -1;
    }

    Fbar_i = F_i/F_i.norm();
    Fbar_b = F_b/F_b.norm();

    f_total = F_i.norm();

    q_d.w() = (1 + Fbar_i.dot(Fbar_b))*(1/sqrt(2*(1+Fbar_i.dot(Fbar_b))));
    q_d.vec() = Fbar_i.cross(Fbar_b)*(1/sqrt(2*(1+Fbar_i.dot(Fbar_b))));

    q_d = q_d*Q_psid;

    Fbardot_i = (Fdot_i/F_i.norm()) - (F_i*(F_i.dot(Fdot_i)))/pow(F_i.norm(),3);

    q_e = q_imu.inverse()*q_d;

    psidot_d = (psi_d - psi_d_prev)*(1/dT);
    psi_d_prev = psi_d;

    Omega_d(0) = Fbar_i.cross(Fbardot_i)(0);
    Omega_d(1) = Fbar_i.cross(Fbardot_i)(1);
    Omega_d(2) = psidot_d;

    M_b = -sgn(q_e.w())*Kp*q_e.vec() - Kd*(Omega - Omega_d);

    }


void ACLMCController::getThrusts(float f_total, Eigen::Vector3f){


}
