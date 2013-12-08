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

const double PI = 4.0*atan(1.0);

int main(int argc, char* argv[])
{
    Scene* scene = new Scene();
    
    Vector3f origin ( 0.0, 0.0, 0.0 );
    Vector3f p1 ( 0.0, 4.0, 0.0 );
    Vector3f p2 ( 4.0, 4.0, 0.0 );
    
    Link l1 = Link(4, 0, p1);
    Link l2 = Link(4, PI/2, p2);
    
    l1.addOuterLink(1);
    l2.addInnerLink(0);
    
    vector<Link> path;
    path.push_back(l1);
    path.push_back(l2);

    Kinematics k = Kinematics(origin, path);
    scene->addKinematics(k);

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
