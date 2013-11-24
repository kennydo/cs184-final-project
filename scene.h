#ifndef SCENE_H
#define SCENE_H

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <math.h>
#include <Eigen/Core>

#include "joint.h"
#include "kinematics.h"

class Scene {
    private:
        float theta; //testing FK
        Joint *root;
    public:
        Scene();
        
        void addRootJoint(Joint*);

        void refreshCamera();
        void draw();

        void drawGrid(float, float, float, float, float);
        void drawLink(Link);
        void drawSkeleton();
        void moveSkeleton(float);
};

#endif
