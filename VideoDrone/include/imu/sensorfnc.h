#ifndef SENSORFNC_H
#define SENSORFNC_H

#include <stdio.h>
#include <math.h>
#include "armadillo"
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "../../include/globals/globalstruct.h"

using namespace Eigen;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void EulertoQuaternion(double roll, double pitch, double yaw, Quaternionf Q);
void gps2ned(float gps_current[3], home myHome, Vector3f NED);



#endif // SENSORFNC_H
