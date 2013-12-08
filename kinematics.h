#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <vector>
#include <math.h>
#include <Eigen/Core>

#include "joint.h"

using namespace Eigen;

//class Joint;
class Kinematics {
    Vector3f origin_;
    vector<Link> path_;
    Kinematics() {}
public:
    
    Kinematics(Vector3f origin, vector<Link> path) : origin_(origin), path_(path) {}
    void solveFKHelper(Link &link, float theta, Vector3f pos);
    void solveFK(Link &link, float theta);

    // The link that is passed in must be an end-effector,
    // ie. link.getOuterJoint().getOuterLink().size() == 0. This method will assert this.
    /*static void solveIK(Link *link, Vector3d delta);

    // It would make sense to use eigen matrix for this, unfortunately, we don't know how big our
    // matrix will be so instread we return a std::vector of Vector3fs.
    static MatrixXd jacobian(vector<Link*> &path, vector<float> &thetas, vector<float> &lengths);*/
    friend class Scene;
};

#endif
