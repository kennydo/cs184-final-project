#include <cassert>
#include <cstdio>
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "kinematics.h"

using namespace Eigen;

#define EPSILON 0.00000001

void Kinematics::solveFKHelper(Link &link, float theta, Vector3f pos) {
    float newTheta = link.getAngle() + theta;
    float length = link.getLength();
   
    Vector3f newPos;
    newPos.x() = pos.x() + length*sin(newTheta);
    newPos.y() = pos.y() + length*cos(newTheta);
    newPos.z() = 0;
   
   link.moveJoint(newPos);
   
    printf("new pos x: %f posx: %f length: %f newTheta: %f\n", newPos.x(), pos.x(), length, newTheta);
    printf("new pos y: %f posy: %f length: %f newTheta: %f\n", newPos.y(), pos.y(), length, newTheta);

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
   
   Vector3f position;
   if (link.getInnerLink() == -1) {
       position = origin_;
   } else {
       printf("supposed to print\n");
       position = path_[link.getInnerLink()].pos();
   }
   
   
   solveFKHelper(link, 0, position);
   
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
/*
void getNewPositionHelper(Link &link, double theta, Vector3f pos, Vector3f &newPosition) {
    
    float newTheta = link.getAngle() + theta;
    float length = link.getLength();
   
    position.x() = (inner->pos()).x() + length*sin(angle);
    position.y() = (inner->pos()).y() + length*cos(angle);
    position.z() = 0.0;
   
   //link.updateAngle(newTheta); rest of the angles should stay the same
   //link.moveJoint(newPos);

    vector<int> outer = link.getOuterLinks();
    
    if ( outer.size() > 0 ) {
        for (unsigned int i = 0; i < outer.size(); i++) {
            solveFKHelper(path_[outer[i]], newTheta, newPos);
        }
    }*/
    
    /*Joint *inner = link->getInnerJoint();
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
    }*/
//}

Vector3f Kinematics::getNewPosition(VectorXf d0_step, vector<Link> &path) {

    //float newTheta = link.getAngle() + theta;
    //link.updateAngle(newTheta);
    
    float length, theta;
   
    Vector3f newPosition(0, 0, 0);
    for (unsigned int i = 0; i < d0_step.size(); i++) {
            theta = path[i].getAngle() + d0_step[i];
            printf("theta: %f\n", theta);
            for (unsigned int j = i; j < path.size(); j++) {
                theta += path[j].getAngle();
                length = path[j].getLength();
                newPosition.x() += length*sin(theta);
                newPosition.y() += length*cos(theta);
            }
    }
    
    newPosition += origin_;
    
    //cout << "new Position\n" << newPosition << endl;
    return newPosition;
}

bool Kinematics::reachedGoal(Vector3f goalPosition, Link link, float &distance) {
    
    Vector3f vDistance = goalPosition - link.pos();
    distance = sqrt(vDistance.dot(vDistance));
    //printf("distance: %f\n", distance);
    if (fabs(distance) < EPSILON) {
        printf("GOAL REACHED\n");
        return true;
    }
    return false;
    
}

void Kinematics::solveIK(Link *link, Vector3f goalPosition) {
    // Assert this is an end effector.
    assert(link->getOuterLinks().size() == 0);

    // Trace our way in, putting previous elements in the front.
    // Make duplicate of current path because we don't want to update
    // the actual skeleton until we've finished the system
    vector<Link> path;
    
    path.insert(path.begin(), *link);
    Link current;
    while (current.getInnerLink() != -1) {
        current = path_[path[0].getInnerLink()];
        path.insert(path.begin(), current);
    }
    
    float currentDistance;
    float step = 1;
    //while (!reachedGoal(goalPosition, path.back(), currentDistance)) {
        // Compute the jacobian on this link.
        MatrixXf jacobian = Kinematics::jacobian(path);
        MatrixXf pinv;
        // TODO: handle false ie. the case where links <= 2
        pseudoInverse(jacobian, pinv);
        //d0 = pseudoInverse * delta
        Vector3f delta = goalPosition - path.back().pos();
        VectorXf d0 = pinv*delta;
        
        // calcuate new point caused by d0
        VectorXf d0_step = d0*step;
        cout << "d0 solved\n" << d0_step << endl;
        Vector3f newPosition = getNewPosition(d0_step, path);
        
       /* //calculate distance from goal of new point
        Vector3f vnewDistance = goalPosition - newPosition;
        double newDistance = sqrt(vnewDistance.dot(vnewDistance));

        //if distance decreased, take step
        // if distance did not decrease, half the step and try again
        if (newDistance < currentDistance) {
            printf("distance shorter\n");
            cout << "solved distance\n" << newPosition << endl;
            for (unsigned int i = 0; i < d0.size(); i++) {
                Kinematics::solveFK(path[i], d0_step[i]);
            }
            
            cout << "FK solved distance\n" << path.back().pos() << endl;
            
            
            //Kinematics::solveIK(path.back(), goalPosition);
        } else if (step/2 > 0) {
            //printf("halve step\n");
            step = step/2;
        } else {
            //printf("no solution\n");
            break;
        }*/
   // }
    
    printf("done\n");
    
    //transfer solved system to be drawn
    link->updateLink(path.back());
    for( unsigned int i = path.size()-2 ; i > 0; i-- ) {
        path_[path[i+1].getInnerLink()].updateLink(path[i]);
    }
    
}

// Helper for jacobian, sums angles of terms i to j
float Kinematics::sumAngles(vector<Link> &path, unsigned int i, unsigned int j)
{
    float sum = 0;
    for(unsigned int u = i; u < j; u++) sum += path[u].getAngle(); //(*angles)[u];
    return sum;
}

MatrixXf Kinematics::jacobian(vector<Link> &path)
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
            float sumThetas = sumAngles(path, 0, j+1);
            float length = path[j].getLength();
            
            toReturn(i, 0) += length * cos(sumThetas);
            toReturn(i, 1) -= length * sin(sumThetas);
        }
        // For now, the z coordinate is zero.
        toReturn(i, 2) = 0.0f;
    }

    return toReturn.transpose();
}
