#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <vector>
#include <math.h>
#include <Eigen/Core>

#include "joint.h"

using namespace Eigen;

class Kinematics {
public:
    static void solveFK(Link *link, double theta);
    
    //void evaluateSteps(float step, Vector3f goalPosition, float currentDistance, vector<Link*> &path, vector<float> &thetas, vector<float> &lengths);

    // The link that is passed in must be an end-effector,
    // ie. link.getOuterJoint().getOuterLink().size() == 0. This method will assert this.
    static void solveIK(Link *link, Vector3d delta);

    // It would make sense to use eigen matrix for this, unfortunately, we don't know how big our
    // matrix will be so instread we return a std::vector of Vector3fs.
    static MatrixXd jacobian(vector<Link*> &path, vector<double> &thetas, vector<double> &lengths);
};

#endif
