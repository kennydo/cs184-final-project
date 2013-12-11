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

Quaternionf createQuaternion(float theta, float tx, float ty, float tz) {
    
    float n = sin(theta/2);
    
    float w = cos(theta/2);
    float x = n*tx;
    float y = n*ty;
    float z = n*tz;
    
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

Vector3f Kinematics::getNewPosition(vector<Quaternionf> &rotations, vector<Link> &path) {
    
     Vector3f g;
     
     Link root = path[0];
     
    if (root.getInnerLink() == -1) {
        g = origin_;
    } else {
        g = path_[root.getInnerLink()].pos();
    }
    
    Vector3f newPosition = root.pos();
    
    for (unsigned int i = 0; i < path.size(); i++) {
    
        Vector3f p_g = newPosition - g;
        newPosition = rotation._transformVector(p_g);
        newPosition = newPosition + g;
        
    }
    
    return newPosition;
    
}

//take step (FK) for IK segment linked to the end effector we are dealing with
// instead of using path_ directly
void Kinematics::takeStep(vector<Quaternionf> &rotations, vector<Link> &path) {
     
     Vector3f g;
     
     Link root = path[0];
    Vector3f g = origin;
    
    for (unsigned int i = 0; i < rotations.size(); i++) {
        
        Link base = path[i];
            
        if (base.getInnerLink() == -1) {
            g = origin_;
        } else {
            g = path[i-1].pos();
        }
        
        for (unsigned int j = i; j < path.size(); j++) {
            Link current = path[j];
            
            Vector3f p_g = current.pos() - g;
            Vector3f translated = theta._transformVector(p_g);
            current.moveJoint(translated + g);
        }
    }
    /*
    for (unsigned int i = 0; i < path.size(); i++) {
        
        Vector3f p_g = path[i].pos() - g;
        Vector3f translated = rotation._transformVector(p_g);
        path[i].moveJoint(translated + g);
        
    }*/
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
        current = path_[path[0].getInnerLink()]; ///this can't be right...
        path.insert(path.begin(), current);
    }
    
    //adds up all d0 so we'd only have to update kinematics only once at the end
    VectorXf d0_final;
    d0_final = VectorXf::Zero(path.size());
    
    
    
    float currentDistance;
    float step = 1;
    while (!reachedGoal(goalPosition, path.back(), currentDistance)) {
        // Compute the jacobian on this link.
        MatrixXf jacobian = Kinematics::jacobian(path, goalPosition);
        MatrixXf pinv;
        // TODO: handle false ie. the case where links <= 2
        pseudoInverse(jacobian, pinv);
        //d0 = pseudoInverse * delta
        Vector3f delta = goalPosition - path.back().pos();
        VectorXf d0 = pinv*delta;
        
        // calcuate new point caused by d0
        VectorXf d0_step = d0*step;
        
        vector<Quaterninof> tempRotations;
        for (unsigned int i = 0; i < d0_step; i++) {
            tempRotations.insert(makeQuaternion(d0_step[i], 1, 1, 1));
        }
        
        Vector3f newPosition = getNewPosition(tempRotations, path);
        
        //calculate distance from goal of new point
        Vector3f vnewDistance = goalPosition - newPosition;
        float newDistance = sqrt(vnewDistance.dot(vnewDistance));

        //if distance decreased, take step
        // if distance did not decrease, half the step and try again
        if (newDistance < currentDistance) {
            takeStep(tempRotations, path);
            d0_final += d0_step;
        } else if (step/2 > 0) {
            step = step/2;
        } else {
            break;
        }
   }

    for (unsigned int i = 0; i < d0_final; i++) {
        rotation = makeQuaternion(d0_final[i], 1, 1, 1));
        current = path[i+1];
        solveFK(path_[current.getInnerLink()], d0_final[i]);
        
    }
    /*
    //solve FK for actual skeleton
    ///need to change vector to quaternion
    for (unsigned int i = 0; i < d0_final.size()-1; i++) {
        current = path[i+1];
        solveFK(path_[current.getInnerLink()], d0_final[i]);
    }
    solveFK(*link, d0_final[d0_final.size()-1]);*/
    
}

/*
// Helper for jacobian, sums angles of terms i to j
float Kinematics::sumAngles(vector<Link> &path, unsigned int i, unsigned int j)
{
    float sum = 0;
    for(unsigned int u = i; u < j; u++) sum += path[u].getAngle(); //(*angles)[u];
    return sum;
}*/

MatrixXf Kinematics::jacobian(vector<Link> &path, Vector3f g)
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
        
        //gives us one column in the jacobian matrix
        Vector3f derivative = a.cross(e-r);
        
        toReturn(0, i) = derivative.x();
        toReturn(1, i) = derivative.y();
        toReturn(2, i) = derivative.z();
        
        cout << "to Return\n" << derivative << endl;
    
    }
    
    return toReturn;
    
    /*// Now that we have all the lengths and thetas
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

    return toReturn.transpose();*/
}
