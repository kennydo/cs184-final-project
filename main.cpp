#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <Eigen/Geometry>
#include <math.h>
#include <stdio.h>
#include "joint.h"
#include "kinematics.h"
#include "objparser.h"
#include "window.h"
#include "scene.h"

const double PI = 4.0*atan(1.0);

int main(int argc, char* argv[])
{

    ParsedObj* obj = NULL;
    if(argc >= 2){
        printf("Attempting to parse obj file input\n");
        obj = ObjParser::parse(argv[1]);

        printf("Centering and scaling obj\n");
        obj->centerAndScale(10.0);
        printf("Centering obj by: %f, %f, %f\n",
                obj->center.x(),
                obj->center.y(),
                obj->center.z());
        printf("Scaling obj by: %f\n", obj->scale);

        printf("Completed parsing\n");
    } else {
        printf("No .obj file passed in\n");
    }
    Skeleton* skeleton = NULL;
    if(argc >= 3){
        printf("Attempting to parse skeleton file input\n");
        skeleton = Skeleton::parse(argv[2]);
        printf("Completed parsing\n");

        Eigen::Vector3f offset = -1 * obj->center;
        printf("Centering by (%f, %f, %f)\n", offset.x(), offset.y(), offset.z());
        skeleton->offset(-1 * obj->center);

        float scale = obj->scale;
        printf("Scaling by %f\n", scale);
        skeleton->scale(obj->scale);
    } else {
        printf("No skeleton file passed in\n");
    }
    /*
    Vector3f origin ( -1.0, 0.0, 0.0 );
    
    Vector3f p1 ( 0.0, 4.0, 0.0 ); // 0
    Vector3f p2 ( 4.0, 4.0, 0.0 ); // 1
    Vector3f p3 ( 4.0, 2.0, 0.0 ); // 2
    
    Vector3f p4 ( 0.0, -4.0, 0.0 ); // 3
    Vector3f p5 ( 4.0, -4.0, 0.0 ); // 4
    Vector3f p6 ( 4.0, -2.0, 0.0 ); // 5

    Quaternionf q1(1, 0, 0, 0);

    Quaternionf q2(sqrt(0.5), 0, 0, sqrt(0.5));

    Quaternionf q3(sqrt(0.5), 0, 0, sqrt(0.5));
    
    Link l0 = Link(0, origin, q1);
    Link l1 = Link(4, p1, q1);
    Link l2 = Link(4, p2, q2);
    Link l3 = Link(2, p3, q3);
    
    
    Link l4 = Link(4, p4, q1);
    Link l5 = Link(4, p5, q1);
    Link l6 = Link(2, p6, q1);
    
    l0.addOuterLink(1);
    l1.addInnerLink(0);
    l1.addOuterLink(2);
    l2.addInnerLink(1);
    l2.addOuterLink(3);
    l3.addInnerLink(2);
    
    l4.addOuterLink(5);
    l5.addInnerLink(4);
    l5.addOuterLink(6);
    l6.addInnerLink(5);
    
    
    vector<Link> path;
    path.push_back(l0);
    path.push_back(l1);
    path.push_back(l2);
    path.push_back(l3);
    path.push_back(l4);
    path.push_back(l5);
    path.push_back(l6);*/
    
    
    
    vector<Link> path;
    for(int i=0; i < int(skeleton->joints.size()); i++){
        path.push_back(* (skeleton->joints[i]));
    }
    /*
    printf("Printing path\n");
    for(int i=0; i < int(path.size()); i++){
        Vector3f position = path[i].pos();
        printf("Link at (%f, %f, %f)\n", position.x(), position.y(), position.z());
        printf("Innerlink: %d\n", path[i].getInnerLink());
        vector<int> outerLinks = path[i].getOuterLinks();
        printf("Outerlinks:\n");
        for(int j=0; j < int(outerLinks.size()); j++){
            printf("\t%d\n", outerLinks[j]);
        }
    }*/

    //Kinematics* kinematics = new Kinematics(origin, path);

    Kinematics*  kinematics = new Kinematics(path[0].pos(), path);

    Scene* scene = new Scene(obj, skeleton, kinematics);

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
