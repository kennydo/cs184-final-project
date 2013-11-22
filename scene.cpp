#include "scene.h"

Scene::Scene(){
    // initialize variables
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
    drawGrid(-10, 10, -10, 10, 1);
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

