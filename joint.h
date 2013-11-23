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
    Joint(Vector3f position) : pos_(position) {}

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
    Link(float length, float theta) : length_(length), theta_(theta) {}
    
    void addInnerJoint(Joint *inner) { j0_ = inner; }
    void addOuterJoint(Joint *outer) { j1_ = outer; }
    
    Joint* getInnerJoint() { return j0_; }
    Joint* getOuterJoint() { return j1_; }
    
    float getLength() { return length_; }
    float getAngle() { return theta_; }
};

class Body
{
private:
    vector<Joint*> joints_;
    vector<Link*> links_;
public:
    Body() {}

    void addJoint(Joint *joint) { joints_.push_back(joint); }
    void addLink(Link *link) { links_.push_back(link); }
};

#endif
