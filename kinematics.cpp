#include "kinematics.h"
#include <cstdio>

/*
 * Given angle (in radians) of joint movement and link
 * calculates new position of joint and all outer joints
 */
void Kinematics::solveFK(Link link, float theta) {
    Joint *inner = link.getInnerJoint();
    Joint *outer = link.getOuterJoint();
    
    float angle = link.getAngle() + theta;
    float length = link.getLength();
    
    Vector3f v;
    v.x() = (inner->pos()).x() + length*sin(angle);
    v.y() = (inner->pos()).y() + length*cos(angle);
    v.z() = 0.0;
    
    outer->moveJoint(v);
    
    printf("new joint position x: %f y: %f z: %f\n", v.x(), v.y(), v.z());
    
    vector<Link*> outerLinks = outer->getOuterLink();
    
    if ( outerLinks.size() > 0 ) {
        for (unsigned int i = 0; i < outerLinks.size(); i++) {
            solveFK(*(outerLinks[i]), angle);
        }
    }

}
