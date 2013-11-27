#include "kinematics.h"
#include <cassert>
#include <cstdio>
#include <cmath>

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

void Kinematics::solveIK(Link *link, Vector3f delta) {
    // Assert this is an end effector.
    assert(link->getOuterJoint()->getOuterLink().size() == 0);

    // Compute the jacobian on this link.
    vector<Vector3f> jacobian = Kinematics::jacobian(link);

    // TODO: (hkothari) what else?
}

// Helper for jacobian, sums angles of terms i to j
float sumAngles(vector<float>* angles, unsigned int i, unsigned int j)
{
    float sum = 0.0f;
    for(int u = i; u < j; u++) sum += (*angles)[u];
    return sum;
}

vector<Vector3f> Kinematics::jacobian(Link *link)
{
    // Trace our way in, putting previous elements in the front.
    vector<Link*> path;
    vector<float> thetas;
    vector<float> lengths;

    path.insert(path.begin(), link);
    thetas.insert(thetas.begin(), link->getAngle());
    lengths.insert(lengths.begin(), link->getLength());

    Joint* innerJoint = link->getInnerJoint();
    Link* innerLink = innerJoint->getInnerLink();

    while(innerLink != NULL)
    {
        path.insert(path.begin(), innerLink);
        thetas.insert(thetas.begin(), innerLink->getAngle());
        lengths.insert(lengths.begin(), innerLink->getLength());

        innerJoint = innerLink->getInnerJoint();
        innerLink = innerJoint->getInnerLink();
    }

    // Now that we have all the lengths and thetas
    // Our matrix is of the form [dpn/dt1 dpn/dt2 ... dpn/dtn]
    // Logic replicated from http://njoubert.com/teaching/cs184_fa08/section/sec13inversekinematicsSOL.pdf (p.4)
    vector<Vector3f> toReturn;
    unsigned int n = path.size();
    for(unsigned int i = 0; i < n; i++)
    {
        // The ith column has n-i terms: 
        Vector3f col;
        for(unsigned int j = i; j < n; j++)
        {
            //l1 cos(θ1) + l2 cos(θ1 + θ2)+ l3 cos(θ1 + θ2 + θ3) 
            float sumThetas = sumAngles(&thetas, 0, j+1);
            col.x() += lengths[j] * cos(sumThetas);
            col.y() -= lengths[j] * sin(sumThetas);
        }
        // For now, the z coordinate is zero.
        col.z() = 0;

        // Since we're working our way from dt1 to dtn we push_back
        toReturn.push_back(col);
    }
    return toReturn;
}
