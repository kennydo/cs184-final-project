#include "scene.h"
#include <cstdio>

Scene::Scene(){
    // initialize variables
    theta = 0;
    renderMode = GL_RENDER;
    mouseButtonPressed = 0;
}

void Scene::addRootJoint(Joint *j) {
    root = j;
}

void Scene::refreshCamera(int x, int y){
    // the x and y are where the click happened.
    // when the renderMode isn't GL_SELECT, those values aren't used for anything
    // so just pass in dummy 0,0
    int viewport[4];

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
    if(renderMode == GL_SELECT){
        glGetIntegerv(GL_VIEWPORT, viewport);
        gluPickMatrix((double) x, (double) y,
                      PICK_TOLERANCE, PICK_TOLERANCE,
                      viewport);
    }

    // we'll probably want to not be using ortho projection
    glOrtho(-0.6, 0.6,
            -0.6, 0.6,
            -1.0, 1.0);

}

void Scene::draw(){
    glInitNames();
    glPushName(0);
    glLoadName(67); // a distinct-looking name for debugging purposes
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

void Scene::rotateSkeleton(float f) {
    theta += f*3.14159/180;
    
    vector<Link*> outerLinks = root->getOuterLink();
    Kinematics::solveFK(*(outerLinks.front()), theta);
}

// Moves the skeleton up and down, obviously this is poorly named..
// we'll work on that.
void Scene::moveSkeleton(float f) {
    // IK can only solve for the end effector, so we want to find the last
    // element and move it.
    Vector3f delta;
    delta.x() = 0.0f;
    delta.y() = f;
    delta.z() = 0.0f;

    // TODO: We're just using the firstmost link for now this is obviously
    // not a real solution.
    Link *current = root->getOuterLink()[0];
    Joint *j = current->getOuterJoint();
    while(j != NULL && j->getOuterLink().size() == 0) {
        current = j->getOuterLink()[0];
        j = current->getOuterJoint();
    }

    Kinematics::solveIK(current, delta);
}

void Scene::onLeftClick(int x, int y) {
    /*
     * the x and y are where the click happened, where
     * x=0 is left side of the screen and
     * y=0 is the bottom of the screen
     */
    printf("Scene::onLeftClick called with x=%d, y=%d\n", x, y);
    mouseButtonPressed = GLUT_LEFT_BUTTON;
    mouseClickStartX = x;
    mouseClickStartY = y;
    GLint numHits;

    glSelectBuffer(PICK_BUFFER_SIZE, pickBuffer);
    renderMode = GL_SELECT;
    glRenderMode(renderMode);
    refreshCamera(x, y);
    draw();

    renderMode = GL_RENDER;
    numHits = glRenderMode(renderMode);
    // if numHits == -1, there was a overflow in the selection buffer
    if (numHits == 0){
        return;
    }

    printf("Scene got %d hits\n", numHits);
    // we successfully hit a named object!
    GLuint numItems, item;
    float zMin, zMax;
    // we only care about the first hit for now
    numItems = pickBuffer[0];
    zMin = pickBuffer[1] / (pow(2, 32) - 1.0);
    zMax = pickBuffer[2] / (pow(2, 32) - 1.0);
    printf("numItems: %d\nzMin: %f\nzMax: %f\n",
           numItems, zMin, zMax);
    for(unsigned int j=0; j<numItems; j++){
        item = pickBuffer[3 + j];
        printf("item: %d\n", item);
    }
    printf("\n");
}

void Scene::onLeftRelease(int x, int y) {
    printf("Scene::onLeftRelease called with x=%d, y=%d\n", x, y);
    mouseButtonPressed = 0;
}

void Scene::onMouseMotion(int x, int y) {
    // this function is called every time the mouse moves while a button is pressed
    int dX = x - mouseClickStartX;
    int dY = y - mouseClickStartY;

    printf("Scene::onMouseMotion called with x=%d, y=%d    dX=%d, dY=%d\n", x, y, dX, dY);
    if(mouseButtonPressed == GLUT_LEFT_BUTTON){

    } else if (mouseButtonPressed == GLUT_RIGHT_BUTTON) {

    }
}
