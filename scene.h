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
#include <Eigen/Geometry>

#include "joint.h"
#include "kinematics.h"
#include "objparser.h"
#include "skeleton.h"

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

        // these are in world coordinates (object)
        Vector3f mouseClickStart;
        Vector3f mousePrevious;

        // this is in screen coordinates
        Vector3f windowPrevious;

        Vector3f translateAmount;
        Vector3f rotateAmount;

        float scaleFactor;

        void mouseToWorldCoordinates(int, int, double&, double&, double&);
        MouseToWorldConverter *converter;

        Kinematics* kinematics;
        ParsedObj* obj;
        Skeleton* skeleton;

        int selectedJointId;
        float hitZ;

        void updateSkeletonJointPositions();

        bool displayGrid;
        bool displaySkeleton;
        bool displayObj;
    public:
        Scene(ParsedObj*, Skeleton*, Kinematics*);

        void refreshCamera(int, int);
        void draw();

        void drawGrid();
        void drawLink(Link);
        void drawSkeleton();
        void drawObj();

        void moveJoint(Vector3f direction);

        void onMouseMotion(int, int);
        void onLeftClick(int, int);
        void onLeftRelease(int, int);

        void onRightClick(int, int);
        void onRightRelease(int, int);

        void onZoomIn();
        void onZoomOut();

        void toggleGrid() { displayGrid = !displayGrid; };
        void toggleSkeleton() { displaySkeleton = !displaySkeleton; };
        void toggleObj() { displayObj = !displayObj; };
};


#endif
