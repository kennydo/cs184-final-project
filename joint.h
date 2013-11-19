#ifndef JOINT_H
#define JOINT_H

#include <Eigen/Core>

using namespace Eigen;

class Joint
{
private:
    Vector3f pos_;
public:
    Joint(Vector3f position) : pos_(position) {}
    
    virtual float angleBetween(Joint *other) = 0;

    Vector3f pos() { return pos_; }
};

#endif
