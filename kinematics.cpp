#include <cassert>
#include <cstdio>
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/SVD>

#include "kinematics.h"

using namespace Eigen;
#define EPSILON 0.00000001

/*
 * Given angle (in radians) of joint movement and link
 * calculates new position of joint and all outer joints
 */
 
 void solveFKHelper(Link *link, float theta) {
     
    Joint *inner = link->getInnerJoint();
    Joint *outer = link->getOuterJoint();
    
    float angle = link->getAngle() + theta;
    float length = link->getLength();
    
    Vector3f v;
    v.x() = (inner->pos()).x() + length*sin(angle);
    v.y() = (inner->pos()).y() + length*cos(angle);
    v.z() = 0.0;
    
    outer->moveJoint(v);
    
    vector<Link*> outerLinks = outer->getOuterLink();
    
    if ( outerLinks.size() > 0 ) {
        for (unsigned int i = 0; i < outerLinks.size(); i++) {
            solveFKHelper(outerLinks[i], angle);
        }
    }
     
 }
 
void Kinematics::solveFK(Link* link, float theta) {
    
    float newTheta = link->getAngle() + theta;
    
    link->updateAngle(newTheta);
    solveFKHelper(link, 0);
   
}


// Compute pseudo inverse, logic from: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
template<typename _Matrix_Type_>
bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double epsilon = std::numeric_limits<double>::epsilon())
{
    if(a.rows() < a.cols())
        return false;
    Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();
    result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0) ).asDiagonal() *
    svd.matrixU().adjoint();
    return true;
}

void getNewPositionHelper(Link *link, float theta, Vector3f &position) {
    
    Joint *inner = link->getInnerJoint();
    Joint *outer = link->getOuterJoint();
    
    float angle = link->getAngle() + theta;
    float length = link->getLength();
    
    position.x() = (inner->pos()).x() + length*sin(angle);
    position.y() = (inner->pos()).y() + length*cos(angle);
    position.z() = 0.0;
    
    vector<Link*> outerLinks = outer->getOuterLink();
    
    if ( outerLinks.size() > 0 ) {
        for (unsigned int i = 0; i < outerLinks.size(); i++) {
            getNewPositionHelper(outerLinks[i], angle, position);
        }
    }
    
}

Vector3f getNewPosition(VectorXf d0_step, vector<Link*> &path, vector<float> &thetas, vector<float> &lengths) {

    Vector3f newPosition(0, 0, 0);
    for (unsigned int i = 0; i < d0_step.size(); i++) {
        getNewPositionHelper(path[i], d0_step[i], newPosition);
    }
    
    return newPosition;
}

bool reachedGoal(Vector3f goalPosition, Link * link, float &distance) {
    
    Vector3f vDistance = goalPosition - (link->getOuterJoint()->pos());
    distance = sqrt(vDistance.dot(vDistance));

    if (fabs(distance) < EPSILON) {
        printf("GOAL REACHED\n");
        /*cout << "goal position\n" << goalPosition << endl;
        cout << "outer joint pos\n" << link->getOuterJoint()->pos() << endl;
        printf("distance: %f\n", distance);*/
        return true;
    }
    return false;
    
}

void Kinematics::solveIK(Link *link, Vector3f goalPosition) {
    // Assert this is an end effector.
    assert(link->getOuterJoint()->getOuterLink().size() == 0);

    
    // Trace our way in, putting previous elements in the front.
    vector<Link*> path;
    vector<float> thetas;
    vector<float> lengths;
    
    path.insert(path.begin(), link);
    thetas.insert(thetas.begin(), link->getAngle());
    lengths.insert(lengths.begin(), link->getLength());

    Joint* innerJoint = link->getInnerJoint();
    Link* innerLink = innerJoint->getInnerLink();

    //int count = 0;
    while(innerLink != NULL)
    {
        path.insert(path.begin(), innerLink);
        thetas.insert(thetas.begin(), innerLink->getAngle());
        lengths.insert(lengths.begin(), innerLink->getLength());

        innerJoint = innerLink->getInnerJoint();
        innerLink = innerJoint->getInnerLink();
    }
    
    float currentDistance;
    float step = 0.05;
    while (!reachedGoal(goalPosition, link, currentDistance)) {
        // Compute the jacobian on this link.
        MatrixXf jacobian = Kinematics::jacobian(path, thetas, lengths);
        MatrixXf pinv;
        // TODO: handle false ie. the case where links <= 2
        pseudoInverse(jacobian, pinv);
        
        //d0 = pseudoInverse * delta
        Vector3f delta = goalPosition - (link->getOuterJoint()->pos());
        VectorXf d0 = pinv*delta;
        
        // calcuate new point caused by d0
        VectorXf d0_step = d0*step;
        Vector3f newPosition = getNewPosition(d0_step, path, thetas, lengths);
        
        //calculate distance from goal of new point
        Vector3f vnewDistance = goalPosition - newPosition;
        float newDistance = sqrt(vnewDistance.dot(vnewDistance));

        //if distance decreased, take step
        // if distance did not decrease, half the step and try again
        if (newDistance < currentDistance) {
            for (unsigned int i = 0; i < d0.size(); i++) {
                Kinematics::solveFK(path[i], d0_step[i]);
            }
            /*cout << "------------------------------\n" << endl;
            cout<< "new position\n" << newPosition <<endl;
            cout<< "FK new position\n" << link->getOuterJoint()->pos() << endl;*/
            Kinematics::solveIK(path.back(), goalPosition);
        } else if (isgreater(step/2, 0.0f)) {
            step = step/2;
        } else {
            return;
        }
    }
    
}

// Helper for jacobian, sums angles of terms i to j
float sumAngles(vector<float>* angles, unsigned int i, unsigned int j)
{
    float sum = 0.0f;
    for(unsigned int u = i; u < j; u++) sum += (*angles)[u];
    return sum;
}

MatrixXf Kinematics::jacobian(vector<Link*> &path, vector<float> &thetas, vector<float> &lengths)
{
    // Now that we have all the lengths and thetas
    // Our matrix is of the form [dpn/dt1 dpn/dt2 ... dpn/dtn]
    // Logic replicated from http://njoubert.com/teaching/cs184_fa08/section/sec13inversekinematicsSOL.pdf (p.4)
    unsigned int n = path.size();
    MatrixXf toReturn = MatrixXf::Constant(n, 3, 0);
    for(unsigned int i = 0; i < n; i++)
    {
        // The ith column has n-i terms: 
        for(unsigned int j = i; j < n; j++)
        {
            //l1 cos(θ1) + l2 cos(θ1 + θ2)+ l3 cos(θ1 + θ2 + θ3) 
            float sumThetas = sumAngles(&thetas, 0, j+1);
            toReturn(i, 0) += lengths[j] * cos(sumThetas);
            toReturn(i, 1) -= lengths[j] * sin(sumThetas);
        }
        // For now, the z coordinate is zero.
        toReturn(i, 2) = 0.0f;
    }

    return toReturn.transpose();
}
