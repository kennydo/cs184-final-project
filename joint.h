#ifndef JOINT_H
#define JOINT_H

#include <vector>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

/*class Link;

class Joint
{
private:
    Vector3d pos_;
    Link *l0_; //inner link
    vector<Link*> l1_; //outer link(s)
    
public:
    Joint(Vector3d position) : pos_(position), l0_(NULL) {}

    void addInnerLink(Link *inner) { l0_ = inner; }
    void addOuterLink(Link *outer) { l1_.push_back(outer); }

    Vector3d pos() { return pos_; }
    Link* getInnerLink() { return l0_; }
    vector<Link*> getOuterLink() { return l1_; }
    void moveJoint(Vector3d newPosition) { pos_ = newPosition; }
};*/

class Link
{
private:
    float length_, theta_;
    Vector3f pos_;
    int l0_;
    vector<int> l1_;
    
    //Quaternionf unit;
    //Quaternionf worldOrient;
    
public:
    Link() {}
    Link(float length, float theta, Vector3f position) : length_(length), theta_(theta), pos_(position), l0_(-1) {}
    
    void addInnerLink(int inner) { l0_ = inner; }
    void addOuterLink(int outer) { l1_.push_back(outer); }
    int getInnerLink() { return l0_; }
    vector<int> getOuterLinks() { return l1_; }
    
    Vector3f pos() { return pos_; }
    float getLength() { return length_; }
    float getAngle() { return theta_; }
    
    void moveJoint(Vector3f pos) { pos_ = pos; }
    void updateAngle(float theta) { theta_ = theta; }
};

/*
class Body
{
private: 
    vector<Link> path_;
public:
    friend Kinematics
};*/

#endif
