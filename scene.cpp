#include "scene.h"
#include <cstdio>

Scene::Scene(){
    // initialize variables
    theta = 0;
}

void Scene::addRootJoint(Joint *j) {
    root = j;
}

void Scene::refreshCamera(){
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    GLfloat light_pos[4] = {0.0, 0.0, -1.0, 0.0};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    glRotatef(30, 1, 0, 0);
    glRotatef(30, 0, 1, 0);

    // we'll probably want to use AABB and figure out scaling
    glScalef(0.05, 0.05, 0.05);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // we'll probably want to not be using ortho projection
    glOrtho(-0.6, 0.6,
            -0.6, 0.6,
            -1.0, 1.0);

}

void Scene::draw(){
    drawSkeleton();
}

void Scene::drawGrid(float xMin, float xMax, float zMin, float zMax, float step){
    float x, z;
    glColor3f(1.0, 1.0, 1.0);
    for(x=xMin; x <= xMax; x += step){
        glBegin(GL_LINES);
        glVertex3f(x, 0, zMin);
        glVertex3f(x, 0, zMax);
        glEnd();
    }

    for(z = zMin; z <= zMax; z += step){
        glBegin(GL_LINES);
        glVertex3f(xMin, 0, z);
        glVertex3f(xMax, 0, z);
        glEnd();
    }
}

void Scene::drawLink(Link link) {
    Joint *j1 = link.getInnerJoint();
    Joint *j2 = link.getOuterJoint();
    
    Vector3f p1 = j1->pos();
    Vector3f p2 = j2->pos();
    
    //printf("vector 1 x: %f y: %f z: %f\n", p1.x(), p1.y(), p1.z());
    //printf("vector 2 x: %f y: %f z: %f\n", p2.x(), p2.y(), p2.z());
    
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINES);
    glVertex3f(p1.x(), p1.y(), p1.z());
    glVertex3f(p2.x(), p2.y(), p2.z());
    glEnd();
    
    vector<Link*> outerLinks = j2->getOuterLink();
    
    if ( outerLinks.size() > 0 ) {
        for (unsigned int i = 0; i < outerLinks.size(); i++) {
            drawLink(*outerLinks[i]);
        }
    }
}

void Scene::drawSkeleton() {
    vector<Link*> outerLinks = root->getOuterLink();
    if ( outerLinks.size() > 0 ) {
        for (unsigned int i = 0; i < outerLinks.size(); i++) {
            drawLink(*outerLinks[i]);
        }
    }
    
}

void Scene::moveSkeleton(float f) {
    theta += f*3.14159/180;
    
    vector<Link*> outerLinks = root->getOuterLink();
    Kinematics::solveFK(*(outerLinks.front()), theta);
}
