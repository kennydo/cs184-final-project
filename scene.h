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
#define PICK_BUFFER_SIZE 512

class Scene {
    private:
        float theta; //testing FK
        GLuint pickBuffer[PICK_BUFFER_SIZE];
        GLenum renderMode; // either GL_RENDER or GL_SELECT
        Joint *root;
    public:
        Scene();
        
        void addRootJoint(Joint*);

        void refreshCamera(int, int);
        void draw();

        void drawGrid(float, float, float, float, float);
        void drawLink(Link);
        void drawSkeleton();

        void rotateSkeleton(float);
        void moveSkeleton(float);

        void onLeftClick(int, int);
        void onLeftRelease(int, int);
};

#endif
