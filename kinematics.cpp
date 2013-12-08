#include <cassert>
#include <cstdio>
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "kinematics.h"

using namespace Eigen;

void Kinematics::solveFKHelper(Link &link, float theta, Vector3f pos) {
    float newTheta = link.getAngle() + theta;
    float length = link.getLength();
   
    Vector3f newPos;
    newPos.x() = pos.x() + length*sin(newTheta);
    newPos.y() = pos.y() + length*cos(newTheta);
    newPos.z() = 0;
   
   //link.updateAngle(newTheta);
   link.moveJoint(newPos);
   /*
   cout << "new theta\t" << newTheta << "\t" << "lenghth" << endl;
   cout << "old position\n" << pos << endl;
   cout << "new position\n" << newPos << endl;
   */
    vector<int> outer = link.getOuterLinks();
    
    if ( outer.size() > 0 ) {
        for (unsigned int i = 0; i < outer.size(); i++) {
            solveFKHelper(path_[outer[i]], newTheta, newPos);
        }
    }
    
    
    
 }
 
void Kinematics::solveFK(Link &link, float theta) {
   
   //update angle for ONLY the link affected
   float newTheta = link.getAngle() + theta;
   link.updateAngle(newTheta);
   
   solveFKHelper(link, 0, origin_);
   
   
   /*float length = link.getLength();
   
   Vector3f newPos;
   newPos.x() = pos.x() + length*sin(newTheta);
   newPos.y() = pos.y() + length*cos(newTheta);
   newPos.z() = 0;
   
   //update angle of joint directly affected
   link.updateAngle(newTheta);
   //move joint
   link.moveJoint(newPos);
   
    vector<int> outer = link.getOuterLinks();
    
    if ( outer.size() > 0 ) {
        for (unsigned int i = 0; i < outer.size(); i++) {
            solveFKHelper(path_[outer[i]], 0, newPos);
        }
    }*/
}

/*

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

void getNewPositionHelper(Link *link, double theta, Vector3d &position) {
    
    Joint *inner = link->getInnerJoint();
    Joint *outer = link->getOuterJoint();
    
    double angle = link->getAngle() + theta;
    double length = link->getLength();
    
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

Vector3d getNewPosition(VectorXd d0_step, vector<Link*> &path, vector<double> &thetas, vector<double> &lengths) {

    Vector3d newPosition(0, 0, 0);
    for (unsigned int i = 0; i < d0_step.size(); i++) {
        getNewPositionHelper(path[i], d0_step[i], newPosition);
    }
    
    return newPosition;
}

bool reachedGoal(Vector3d goalPosition, Link * link, double &distance) {
    
    Vector3d vDistance = goalPosition - (link->getOuterJoint()->pos());
    distance = sqrt(vDistance.dot(vDistance));

    if (abs(distance)<2.0*numeric_limits<double>::epsilon()) {
        printf("GOAL REACHED\n");
        return true;
    }
    return false;
    
}

void Kinematics::solveIK(Link *link, Vector3d goalPosition) {
    // Assert this is an end effector.
    assert(link->getOuterJoint()->getOuterLink().size() == 0);

    // Trace our way in, putting previous elements in the front.
    vector<Link*> path;
    vector<double> thetas;
    vector<double> lengths;
    
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
    
    double currentDistance;
    double step = 0.1;
    while (!reachedGoal(goalPosition, link, currentDistance)) {
        // Compute the jacobian on this link.
        MatrixXd jacobian = Kinematics::jacobian(path, thetas, lengths);
        MatrixXd pinv;
        // TODO: handle false ie. the case where links <= 2
        pseudoInverse(jacobian, pinv);
        
        //d0 = pseudoInverse * delta
        Vector3d delta = goalPosition - (link->getOuterJoint()->pos());
        VectorXd d0 = pinv*delta;
        
        // calcuate new point caused by d0
        VectorXd d0_step = d0*step;
        Vector3d newPosition = getNewPosition(d0_step, path, thetas, lengths);
        
        //calculate distance from goal of new point
        Vector3d vnewDistance = goalPosition - newPosition;
        double newDistance = sqrt(vnewDistance.dot(vnewDistance));

        //if distance decreased, take step
        // if distance did not decrease, half the step and try again
        if (newDistance < currentDistance) {
            for (unsigned int i = 0; i < d0.size(); i++) {
                Kinematics::solveFK(path[i], d0_step[i]);
            }
            Kinematics::solveIK(path.back(), goalPosition);
        } else if (step/2 > 0) {
            cout << "step\t" << step << endl;
            step = step/2;
        } else {
            //cout << "step size too small" << endl;
            return;
        }
    }
    
}

// Helper for jacobian, sums angles of terms i to j
double sumAngles(vector<double>* angles, unsigned int i, unsigned int j)
{
    double sum = 0;
    for(unsigned int u = i; u < j; u++) sum += (*angles)[u];
    return sum;
}

MatrixXd Kinematics::jacobian(vector<Link*> &path, vector<double> &thetas, vector<double> &lengths)
{
    // Now that we have all the lengths and thetas
    // Our matrix is of the form [dpn/dt1 dpn/dt2 ... dpn/dtn]
    // Logic replicated from http://njoubert.com/teaching/cs184_fa08/section/sec13inversekinematicsSOL.pdf (p.4)
    unsigned int n = path.size();
    MatrixXd toReturn = MatrixXd::Constant(n, 3, 0);
    for(unsigned int i = 0; i < n; i++)
    {
        // The ith column has n-i terms: 
        for(unsigned int j = i; j < n; j++)
        {
            //l1 cos(θ1) + l2 cos(θ1 + θ2)+ l3 cos(θ1 + θ2 + θ3) 
            double sumThetas = sumAngles(&thetas, 0, j+1);
            toReturn(i, 0) += lengths[j] * cos(sumThetas);
            toReturn(i, 1) -= lengths[j] * sin(sumThetas);
        }
        // For now, the z coordinate is zero.
        toReturn(i, 2) = 0.0f;
    }

    return toReturn.transpose();
}*/
