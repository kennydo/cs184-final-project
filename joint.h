#ifndef JOINT_H
#define JOINT_H

#include <vector>

#include <Eigen/Core>

using namespace std;
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

class Link
{
private:
    Link *parent_;
    float length_;
    Joint *j0_;
    Joint *j1_;
public:
    Link(Link *parent, float length, Joint *j0, Joint *j1)
        : length_(length), j0_(j0), j1_(j1) {}
};

#endif
