#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <stdio.h>
#include "joint.h"
#include "kinematics.h"
#include "window.h"
#include "scene.h"



int main(int argc, char* argv[])
{
    Scene* scene = new Scene();
    
    Vector3f p1 ( 0.0, 0.0, 0.0 );
    Vector3f p2 ( 0.0, 4.0, 0.0 );
    Vector3f p3 ( 4.0, 4.0, 0.0 );
    
    Joint j1 = Joint(p1);
    Joint j2 = Joint(p2);
    Joint j3 = Joint(p3);
    
    Link L12 = Link(4, 0);
    Link L23 = Link(4, 3.14159/2);

    j1.addOuterLink(&L12);
    j2.addInnerLink(&L12);
    j2.addOuterLink(&L23);
    j3.addInnerLink(&L23);
    
    L12.addInnerJoint(&j1);
    L12.addOuterJoint(&j2);
    L23.addInnerJoint(&j2);
    L23.addOuterJoint(&j3);

    scene->addRootJoint(&j1);
    scene->addEndEffector(&j3);

    glutInit(&argc, argv);

    // set up our Window object
    Window::init(800, 640, "CS 184 Kinematics Project", 0, 0, scene);

    // glut only takes static functions,
    // so I had to change the Window's methods to be static
    // so we'll have to hook up our renderer to the
    // Window::display() method using some sort of
    // static variable (in Window, maybe?)

    // disable buffering so we can printf() freely
    setbuf(stdout, NULL);

    // infinite loop that keeps drawing and everything
    glutMainLoop();

    return 0;
}
