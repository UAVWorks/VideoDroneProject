#ifndef SENSORFNC_H
#define SENSORFNC_H

#include <stdio>
#include <math.h>
#include "armadillo"
#include <boost/math/quaternion.hpp>

#include "../../include/globals/globalstruct.h"

void EulertoQuaternion(double roll, double pitch, double yaw, quaternion<double> Q);
void gps2ned(float gps_current[3], home myHome, float NED[3]);

#endif // SENSORFNC_H
