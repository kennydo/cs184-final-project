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
#include "objparser.h"

// picking tolerance in pixels
#define PICK_TOLERANCE 10
#define PICK_BUFFER_SIZE 512

class MouseToWorldConverter {
    private:
        GLdouble modelViewMatrix[16];
        GLdouble projectionMatrix[16];
        GLint viewport[16];
    public:
        MouseToWorldConverter();
        void convert(int, int, double&, double&, double&);
};

class Scene {
    private:
        GLuint pickBuffer[PICK_BUFFER_SIZE];
        GLenum renderMode; // either GL_RENDER or GL_SELECT
        GLenum mouseButtonPressed;
        double mouseClickStartX, mouseClickStartY; // in world coordinates
        double mousePreviousX, mousePreviousY; // in world coordiantes

        double translateX, translateY; // in world coordinates

        void mouseToWorldCoordinates(int, int, double&, double&, double&);
        MouseToWorldConverter *converter;
        
        Kinematics k;
        int endEffector;
        Vector3f delta;
        ParsedObj* obj;
    public:
        Scene(ParsedObj*);
        
        void addKinematics(Kinematics);
        void addEndEffector(int);

        void refreshCamera(int, int);
        void draw();

        void drawGrid();
        void drawLink(Link);
        void drawSkeleton();
        void drawObj();

        void rotateSkeleton(float);
        void moveSkeleton(float x, float y, float z);

        void onMouseMotion(int, int);
        void onLeftClick(int, int);
        void onLeftRelease(int, int);
};


#endif
