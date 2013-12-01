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

// picking tolerance in pixels
#define PICK_TOLERANCE 10
#define PICK_BUFFER_SIZE 128

class Scene {
    private:
        float theta; //testing FK
        unsigned int pickBuffer[PICK_BUFFER_SIZE];
        int renderMode; // either GL_RENDER or GL_SELECT
        Joint *root;
    public:
        Scene();
        
        void addRootJoint(Joint*);

        void refreshCamera();
        void draw();

        void drawGrid(float, float, float, float, float);
        void drawLink(Link);
        void drawSkeleton();

        void rotateSkeleton(float);
        void moveSkeleton(float);

        int getNumClickHits(int, int);
};

#endif
