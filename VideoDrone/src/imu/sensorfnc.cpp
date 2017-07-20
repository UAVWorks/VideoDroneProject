

#include "../../include/imu/sensorfnc.h"

void EulertoQuaternion(double roll, double pitch, double yaw, quaternion<double> Q)
{
        double t0 = std::cos(yaw * 0.5);
        double t1 = std::sin(yaw * 0.5);
        double t2 = std::cos(roll * 0.5);
        double t3 = std::sin(roll * 0.5);
        double t4 = std::cos(pitch * 0.5);
        double t5 = std::sin(pitch * 0.5);

        Q.R_component_1() = t0 * t2 * t4 + t1 * t3 * t5;
        Q.R_component_2() = t0 * t3 * t4 - t1 * t2 * t5;
        Q.R_component_3() = t0 * t2 * t5 + t1 * t3 * t4;
        Q.R_component_4() = t1 * t2 * t4 - t0 * t3 * t5;

}

void gps2ned(float gps_current[3], home myHome, float NED[3]){


}
