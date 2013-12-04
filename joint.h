#ifndef JOINT_H
#define JOINT_H

#include <vector>

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

class Link;

class Joint
{
private:
    Vector3f pos_;
    Link *l0_; //inner link
    vector<Link*> l1_; //outer link(s)
    
public:
    Joint(Vector3f position) : pos_(position), l0_(NULL) {}

    void addInnerLink(Link *inner) { l0_ = inner; }
    void addOuterLink(Link *outer) { l1_.push_back(outer); }

    Vector3f pos() { return pos_; }
    Link* getInnerLink() { return l0_; }
    vector<Link*> getOuterLink() { return l1_; }
    void moveJoint(Vector3f newPosition) { pos_ = newPosition; }
};

class Link
{
private:
    float length_, theta_;
    Joint *j0_; //inner joint
    Joint *j1_; //outer joint
public:
    Link(float length, float theta) : length_(length), theta_(theta), j0_(NULL), j1_(NULL) {}
    
    void addInnerJoint(Joint *inner) { j0_ = inner; }
    void addOuterJoint(Joint *outer) { j1_ = outer; }
    void updateAngle(float theta) { theta_ = theta; }
    
    Joint* getInnerJoint() { return j0_; }
    Joint* getOuterJoint() { return j1_; }
    
    float getLength() { return length_; }
    float getAngle() { return theta_; }
};

#endif
