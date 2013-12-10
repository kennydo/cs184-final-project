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


void printQuaternion(Quaternionf q) {
    
    printf("quaternion: x: %f y: %f z: %f w: %f\n", q.x(), q.y(), q.z(), q.w());
    
}
/*
// pos = position of parent link
// theta = sum of all angles of parents
// link = current link we're solving the angle for
void Kinematics::solveFKHelper(Link &link, Quaternionf theta, Vector3f pos) {
    
    Quaternionf newTheta = link.getAngle()*theta;
    //float length = link.getLength();
    printf("new theta\n");
    printQuaternion(newTheta);
    
    cout << "OLD POSITION\n" << pos << endl;
   
    Vector3f newPos = newTheta._transformVector(link.pos());
    newPos += pos;
    cout << "NEW POSITION\n" << newPos << endl;
    
    //newPos.x() = pos.x() + length*sin(newTheta);
    //newPos.y() = pos.y() + length*cos(newTheta);
    //newPos.z() = 0;
   
   link.moveJoint(newPos);
   
   vector<int> outer = link.getOuterLinks();
    
    if ( outer.size() > 0 ) {
        for (unsigned int i = 0; i < outer.size(); i++) {
            solveFKHelper(path_[outer[i]], newTheta, newPos);
        }
    }
    
}*/

void Kinematics::solveFKHelper(Link &link, Quaternionf theta, Vector3f g) {
    
    Vector3f p_g = link.pos() - g;
    Vector3f translated = theta._transformVector(p_g);
    link.moveJoint(translated + g);
    
    vector<int> outer = link.getOuterLinks();
    
    if ( outer.size() > 0 ) {
        for (unsigned int i = 0; i < outer.size(); i++) {
            solveFKHelper(path_[outer[i]], theta, g);
        }
    }
    
}

void Kinematics::solveFK(Link &link, Quaternionf theta) {
    
    // http://math.stackexchange.com/questions/18382/quaternion-and-rotation-about-an-origin-and-an-arbitrary-axis-origin-help
    // to use quaternion Q to rotate point P with respect to translated origin G: P' = Q(P-G)Q' + G
    
    
    Vector3f g;
    if (link.getInnerLink() == -1) {
        g = origin_;
    } else {
        g = path_[link.getInnerLink()].pos();
    }
    
    solveFKHelper(link, theta, g);
    
    
    /*printf("--------------------------------------------SOLVING FK ---------------------------------------\n");
    printf("theta\n");
    printQuaternion(theta);
    
    //update angle of link moved
    Quaternionf newTheta = link.getAngle()*theta;
    printf("link angle\n");
    printQuaternion(link.getAngle());
    link.updateAngle(newTheta);
    printf("NEW THETA\n");
    printQuaternion(newTheta);
    //in order to get final position of link we have to sum
    //up all of the thetas from the root to the current link
    Link current = link;
    Quaternionf sumTheta;
    sumTheta.setIdentity();
    
    // watch out for the order by which we're multiplying quaternions
    // to rotate a vector v by quaternion q then q'
    // (v^q)^q' = ((qq')^-1)*v*(qq')
    while (current.getInnerLink() != -1) {
        sumTheta *= path_[current.getInnerLink()].getAngle();
        current = path_[current.getInnerLink()];
    }
    
    //get the position of parent link
    Vector3f position(0, 0, 0);
    if (link.getInnerLink() == -1) {
        position = origin_;
    } else {
        position = path_[link.getInnerLink()].pos();
    }
    
    solveFKHelper(link, sumTheta, position);*/
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

Vector3f Kinematics::getNewPosition(VectorXf d0_step, vector<Link> &path) {

    //float newTheta = link.getAngle() + theta;
    //link.updateAngle(newTheta);
    
    float length;
   
    Vector3f newPosition(0, 0, 0);
    if (path[0].getInnerLink() == -1) {
        newPosition += origin_;
    } else {
        newPosition += path_[path[0].getInnerLink()].pos();
    }
    
    //sum theta from root link
    float theta = 0;
    for (unsigned int i = 0; i < d0_step.size(); i++) {
            
            length = path[i].getLength();
            theta += d0_step[i];
            theta += path[i].getAngle();
            
            newPosition.x() += length*sin(theta);
            newPosition.y() += length*cos(theta);
            
            //printf("new x: %f new y: %f \n", newPosition.x(), newPosition.y());
    }
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

//take step (FK) for IK segment linked to the end effector we are dealing with
// instead of using path_ directly
void Kinematics::takeStep(VectorXf d0_step, vector<Link> &path) {
    
    float length, angle;
   
    Vector3f newPosition(0, 0, 0);
    if (path[0].getInnerLink() == -1) {
        newPosition += origin_;
    } else {
        newPosition += path_[path[0].getInnerLink()].pos();
    }
    
    //sum theta from root link
    float theta = 0;
    Link current;
    //p3x = ORIGIN + length1*sin(d01 + 01) + length2*sin(d01 + 01 + d02 + 02) + length3*sin(d01 + 01 + d02 + 02 + d03 + 03)
    for (unsigned int i = 0; i < d0_step.size(); i++) {
            current = path[i];
            angle = current.getAngle();
            length = current.getLength();
            
            theta += d0_step[i];
            theta += angle;
            
            newPosition.x() += length*sin(theta);
            newPosition.y() += length*cos(theta);
            
            path[i].updateAngle(d0_step[i] + angle);
            path[i].moveJoint(newPosition);
            //printf("UPDATING ANGLE TO: %f\n", current.getAngle());
            //printf("UPDATING JOINT POSITION TO\n");
            //cout << newPosition << endl;
    }
    
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
    
    //adds up all d0 so we'd only have to update kinematics only once at the end
    VectorXf d0_final;
    d0_final = VectorXf::Zero(path.size());
    
    float currentDistance;
    float step = 1;
    while (!reachedGoal(goalPosition, path.back(), currentDistance)) {
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
        Vector3f newPosition = getNewPosition(d0_step, path);
        
        //calculate distance from goal of new point
        Vector3f vnewDistance = goalPosition - newPosition;
        float newDistance = sqrt(vnewDistance.dot(vnewDistance));

        //if distance decreased, take step
        // if distance did not decrease, half the step and try again
        if (newDistance < currentDistance) {
            takeStep(d0_step, path);
            d0_final += d0_step;
        } else if (step/2 > 0) {
            step = step/2;
        } else {
            break;
        }
   }
    
    //solve FK for actual skeleton
    for (unsigned int i = 0; i < d0_final.size()-1; i++) {
        current = path[i+1];
        solveFK(path_[current.getInnerLink()], d0_final[i]);
    }
    solveFK(*link, d0_final[d0_final.size()-1]);
    
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
}*/
