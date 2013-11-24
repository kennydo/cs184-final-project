#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <vector>
#include <math.h>
#include <Eigen/Core>

#include "joint.h"

using namespace Eigen;

class Kinematics {
public:
    static void solveFK(Link link, float theta);
    // It would make sense to use eigen matrix for this, unfortunately, we don't know how big our
    // matrix will be so instread we return a std::vector of Vector3fs.
    static vector<Vector3f> jacobian(Link link, float theta_current);
};

#endif
