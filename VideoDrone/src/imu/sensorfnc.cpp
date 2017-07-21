

#include "../../include/imu/sensorfnc.h"

void EulertoQuaternion(double roll, double pitch, double yaw, Eigen::Quaternionf Q)
{
    Q = AngleAxisf(roll, Vector3f::UnitX())
      * AngleAxisf(pitch,  Vector3f::UnitY())
      * AngleAxisf(yaw, Vector3f::UnitZ());

    //std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
}

void gps2ned(float gps_current[3], home myHome, Vector3f NED){


}
