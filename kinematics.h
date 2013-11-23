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

};

#endif
