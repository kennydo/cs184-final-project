#include <cassert>
#include <cstdio>
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/SVD>

#include "kinematics.h"

using namespace Eigen;

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
    
    vector<Link*> outerLinks = outer->getOuterLink();
    
    if ( outerLinks.size() > 0 ) {
        for (unsigned int i = 0; i < outerLinks.size(); i++) {
            solveFK(*(outerLinks[i]), angle);
        }
    }

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

void updateJoints(vector<Link*> &path, vector<float> &thetas, vector<float> &lengths, VectorXf d0) {
    
    Vector3f v;
    v.z() = 0.0;
    for (unsigned int i = 0; i < d0.size(); i++) {
        thetas[i] += d0[i];
        
        Joint *inner = (path[i])->getInnerJoint();
        Joint *outer = (path[i])->getOuterJoint();

        v.x() = (inner->pos()).x() + lengths[i]*sin(thetas[i]);
        v.y() = (inner->pos()).y() + lengths[i]*cos(thetas[i]);
        
        outer->moveJoint(v);
    }
    
}

void evaluateSteps(float step, Vector3f goalPosition, float currentDistance, vector<Link*> &path, vector<float> &thetas, vector<float> &lengths) {
    
    // Compute the jacobian on this link.
    MatrixXf jacobian = Kinematics::jacobian(path, thetas, lengths);
    MatrixXf pinv;
    // TODO: handle false ie. the case where links <= 2
    pseudoInverse(jacobian, pinv);
    
    //d0 = pseudoInverse * delta
    VectorXf d0 = pinv*goalPosition;

    // calcuate new point caused by d0
    Vector3f newPosition(0, 0, 0);
    float total_theta = 0;
    
    for (unsigned int i = 0; i < d0.size(); i++) {
        Joint *inner = (path[i])->getInnerJoint();
        
        total_theta = total_theta + d0[i] + thetas[i];
        
        newPosition.x() += (inner->pos()).x() + lengths[i]*sin(total_theta);
        newPosition.y() += (inner->pos()).y() + lengths[i]*cos(total_theta);
    }
    
    //calculate distance from goal of new point
    Vector3f vnewDistance = goalPosition - newPosition;
    float newDistance = sqrt(vnewDistance.dot(vnewDistance));

    //if distance decreased, take step
    // if distance did not decrease, half the step and try again
    if (newDistance < currentDistance) {
        printf("TAKE STEP currentDistance: %f  newDistance: %f\n", currentDistance, newDistance);
        /*for (unsigned int i = 0; i < d0.size(); i++) {
            Kinematics::solveFK(*(path[i]), d0[i]);
        }*/
        
        updateJoints(path, thetas, lengths, d0);
        
        evaluateSteps(step, goalPosition, newDistance, path, thetas, lengths);
    } else if (isgreater(step/2, 0.00001f)) {
        printf("half step currentDistance: %f  newDistance: %f\n", currentDistance, newDistance);
        evaluateSteps(step/2, goalPosition, currentDistance, path, thetas, lengths);
    } else {
        return;
    }
    
}

void Kinematics::solveIK(Link *link, Vector3f goalPosition) {
    // Assert this is an end effector.
    assert(link->getOuterJoint()->getOuterLink().size() == 0);
    Vector3f vCurrentDistance = goalPosition - link->getOuterJoint()->pos();
    float currentDistance = sqrt(vCurrentDistance.dot(vCurrentDistance));

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
    
    float step = 5*3.14159/180;
    
    evaluateSteps(step, goalPosition, currentDistance, path, thetas, lengths);
    
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
