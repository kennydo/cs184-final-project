#ifndef JOINT_H
#define JOINT_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;



class Link
{
private:
    float length_, theta_;
    Vector3f pos_;
    int l0_;
    vector<int> l1_;
    Quaternionf angle_;
    
public:
    Link() {}
    Link(float length, Vector3f pos, Quaternionf angle) : length_(length), pos_(pos), angle_(angle), l0_(-1) {}
    
    void addInnerLink(int inner) { l0_ = inner; }
    void addOuterLink(int outer) { l1_.push_back(outer); }
    int getInnerLink() { return l0_; }
    vector<int> getOuterLinks() { return l1_; }
    
    Vector3f pos() { return pos_; }
    float getLength() { return length_; }
    Quaternionf getAngle() { return angle_; }
    
    void moveJoint(Vector3f pos) { pos_ = pos; }
    void updateAngle(Quaternionf angle) { angle_ = angle; }
    
    void updateLink(Link link) {
        length_ = link.getLength();
        pos_ = link.pos();
        l0_ = link.getInnerLink();
        l1_ = link.getOuterLinks();
        angle_ = link.getAngle();
    }
};

#endif
