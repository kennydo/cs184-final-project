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

Quaternionf createQuaternion(float theta, Vector3f axis) {
    
    float n = sin(theta/2);
    
    float w = cos(theta/2);
    float x = n*axis.x();
    float y = n*axis.y();
    float z = n*axis.z();
    
    Quaternionf q(w, x, y, z);
    
    return q;
    
    
}

//
// Kinematics
//

void Kinematics::solveFKHelper(Link &link, Quaternionf theta, Vector3f g) {
    
    
    // http://math.stackexchange.com/questions/18382/quaternion-and-rotation-about-an-origin-and-an-arbitrary-axis-origin-help
    // to use quaternion Q to rotate point P with respect to translated origin G: P' = Q(P-G)Q' + G
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
    
    Vector3f g;
    if (link.getInnerLink() == -1) {
        g = origin_;
    } else {
        g = path_[link.getInnerLink()].pos();
    }
    
    solveFKHelper(link, theta, g);

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


void Kinematics::solveFKHelper(Link &link, Quaternionf theta, Vector3f g) {
    
    
    // http://math.stackexchange.com/questions/18382/quaternion-and-rotation-about-an-origin-and-an-arbitrary-axis-origin-help
    // to use quaternion Q to rotate point P with respect to translated origin G: P' = Q(P-G)Q' + G
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
    
    Vector3f g;
    if (link.getInnerLink() == -1) {
        g = origin_;
    } else {
        g = path_[link.getInnerLink()].pos();
    }
    
    solveFKHelper(link, theta, g);

}
*/



Vector3f Kinematics::getNewPosition(vector<Quaternionf> &rotations, vector<Link> &path) {
    
    ///root gives a frame of which point we're rotating around
    Link link;
    Quaternionf currentQuat;
    Vector3f g, p_g, translated;
    
    ///temporarily hold positions of "updated" joints
    vector<Vector3f> updatedPositions(path.size());
    
    for (unsigned int i = 0; i < path.size(); i++) {
        updatedPositions[i] = path[i].pos();
    }
    
    for (unsigned int i = 0; i < rotations.size(); i++ ) {
        
        link = path[i];
        currentQuat = rotations[i];
        
        if (link.getInnerLink() == -1) {
            g = origin_;
        } else {
            g = updatedPositions[i-1];
        }
        
        
        for (unsigned int j = i; j < path.size(); j++ ) {
        
            p_g = updatedPositions[j] - g;
            translated = currentQuat._transformVector(p_g);
            updatedPositions[j] = translated + g;
        }
    }
    
    return updatedPositions.back();
    
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
    //-----------------------------------
    reachedGoal(goalPosition, path.back(), currentDistance);
    //-----------------------------------
    //while (!reachedGoal(goalPosition, path.back(), currentDistance)) {
        // Compute the jacobian on this link.
        
        vector<Vector3f> rotAxis; ///a list of optimal rotation axis for solved d0
        MatrixXf jacobian = Kinematics::jacobian(path, rotAxis, goalPosition);
        MatrixXf pinv;
        // TODO: handle false ie. the case where links <= 2
        pseudoInverse(jacobian, pinv);
        //d0 = pseudoInverse * delta
        Vector3f delta = goalPosition - path.back().pos();
        VectorXf d0 = pinv*delta;
        
        // calcuate new point caused by d0
        VectorXf d0_step = d0*step;
        
        cout << "jacobian\n" << jacobian << endl;
        cout << "d0_step\n" << d0_step << endl;
        
        vector<Quaternionf> rotations;
        ///create quaternions with proper rotational axis
        for (unsigned int i = 0; i < d0_step.size(); i++) {
            rotations.push_back(createQuaternion(d0_step[i], rotAxis[i]));
        }
        
        Vector3f newPosition = getNewPosition(rotations, path);
        
        //calculate distance from goal of new point
        Vector3f vnewDistance = goalPosition - newPosition;
        float newDistance = sqrt(vnewDistance.dot(vnewDistance));

        //if distance decreased, take step
        // if distance did not decrease, half the step and try again
        if (newDistance < currentDistance) {
            ///you have to take the step with IK because of the changing rotAxis for every jacobian
            cout << "newPosition\n" << newPosition << endl;
            printf("currentDistance: %f newDistance: %f\n", currentDistance, newDistance);
            unsigned int rotationSize = rotations.size();
            for (unsigned int i = 0; i < rotationSize - 1; i++) {
                solveFK(path_[path[i+1].getInnerLink()], rotations[i]);
            }
            solveFK(*link, rotations[rotationSize-1]);
            
            cout << "actual solved position\n" << link->pos() << endl;
            
        } else if (step/2 > 0) {
            step = step/2;
        } else {
           //break;
        }
   //}
}

MatrixXf Kinematics::jacobian(vector<Link> &path, vector<Vector3f> &rotAxis, Vector3f g)
{
    
    //a'.cross(e-r')
    //a' = unit length rotational axis
    //r' = position of joint pivot
    //e' = end effector
    
    //a' = ((e-r').cross(g-r')).normalize
    
    unsigned int n = path.size();
    MatrixXf toReturn = MatrixXf::Constant(3, n, 0);
    Vector3f e, r, a;
    
    e = (path.back()).pos();
    
    for (unsigned int i = 0; i < n; i++) {
        
        if (path[i].getInnerLink() == -1) {
            r = origin_;
        } else {
            r = path[i-1].pos();
        }
        
        a = ((e-r).cross(g-r)).normalized();
        
        rotAxis.push_back(a);
        
        //gives us one column in the jacobian matrix
        Vector3f derivative = a.cross(e-r);
        
        toReturn(0, i) = derivative.x();
        toReturn(1, i) = derivative.y();
        toReturn(2, i) = derivative.z();
    
    }
    
    return toReturn;
}
