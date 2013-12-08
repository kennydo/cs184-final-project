#include "scene.h"
#include <cstdio>
#include <iostream>
#include <cmath>

Scene::Scene(){
    // initialize variables
    renderMode = GL_RENDER;
    mouseButtonPressed = 0;
    translateX = 0;
    translateY = 0;

    mousePreviousX = 0;
    mousePreviousY = 0;
    delta = Vector3f(0, 0, 0);
}

void Scene::addKinematics(Kinematics kinematics) {
    k = kinematics;
}

void Scene::refreshCamera(int mouseX, int mouseY){
    // the x and y are where the click happened.
    // when the renderMode isn't GL_SELECT, those values aren't used for anything
    // so just pass in dummy 0,0
    int viewport[4];
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    GLfloat light_pos[4] = {0.0, 0.0, -1.0, 0.0};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    glTranslatef(translateX, translateY, 0);

   //glRotatef(30, 1, 0, 0);
   //glRotatef(30, 0, 1, 0);


    // we'll probably want to use AABB and figure out scaling
    glScalef(0.05, 0.05, 0.05);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(renderMode == GL_SELECT){
        glGetIntegerv(GL_VIEWPORT, viewport);
        gluPickMatrix((double) mouseX, (double) mouseY,
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
    drawGrid();

}

void Scene::drawGrid() {
    
    glPushAttrib(GL_ENABLE_BIT); 
    
    glLineStipple(1, 0xAAAA);
    glEnable(GL_LINE_STIPPLE);
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINES);
    glVertex3f(-100, 0, 0);
    glVertex3f(100, 0, 0);
    glEnd();
    
    glBegin(GL_LINES);
    glVertex3f(0, -100, 0);
    glVertex3f(0, 100, 0);
    glEnd();
    
    glBegin(GL_LINES);
    glVertex3f(0, 0, -100);
    glVertex3f(0, 0, 100);
    glEnd();
    
    glPopAttrib();
}

void Scene::drawSkeleton() {
    Vector3f p1, p2;
    Link link;
    
    for (unsigned int i = 0; i < k.path_.size(); i++) {
        link = (k.path_)[i];
        
        //if root link, then first point is origin
        if (link.getInnerLink() == -1)  {
            p1 = k.origin_;
        } else { // otherwise first point is inner link's position
            p1 = (k.path_[(link.getInnerLink())]).pos();
        }
        
        //second point is just current link's position
        p2 = link.pos();
    
        //draw the link
        glColor3f(1.0, 1.0, 1.0);
        glBegin(GL_LINES);
        glVertex3f(p1.x(), p1.y(), p1.z());
        glVertex3f(p2.x(), p2.y(), p2.z());
        glEnd();
    
    }
    
}

void Scene::rotateSkeleton(float f) {
    
    float theta = f*3.14159/180;
    k.solveFK(k.path_.front(), theta);
    

}
/*

// Moves the skeleton up and down, obviously this is poorly named..
// we'll work on that.
void Scene::moveSkeleton(double f) {
    // IK can only solve for the end effector, so we want to find the last
    // element and move it.
    delta.y() += f;
    
    cout << "-------------------------------\n delta\n" << delta << endl;
    
    cout << "before IK\n" << endEffector->pos() <<endl;
    //Vector3f goalPosition = endEffector->pos() + delta;
    cout << "goalPosition\n" << delta << endl;
    Kinematics::solveIK(endEffector->getInnerLink(), delta);
    cout << "final position\n" << endEffector->pos() << endl;
}*/

void Scene::onLeftClick(int mouseX, int mouseY) {
    /*
     * the x and y are where the click happened, where
     * x=0 is left side of the screen and
     * y=0 is the bottom of the screen
     */
    converter = MouseToWorldConverter();

    double x, y, z;
    converter.convert(mouseX, mouseY, x, y, z);
    mouseClickStartX = x;
    mouseClickStartY = y;
    mousePreviousX = mouseClickStartX;
    mousePreviousY = mouseClickStartY;
    printf("Scene::onLeftClick called with x=%f, y=%f\n", x, y);
    mouseButtonPressed = GLUT_LEFT_BUTTON;
    GLint numHits;

    glSelectBuffer(PICK_BUFFER_SIZE, pickBuffer);
    renderMode = GL_SELECT;
    glRenderMode(renderMode);
    refreshCamera(mouseX, mouseY);
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

void Scene::onLeftRelease(int mouseX, int mouseY) {
    double x, y, z;
    converter.convert(mouseX, mouseY, x, y, z);
    printf("Scene::onLeftRelease called with x=%f, y=%f\n", x, y);
    mouseButtonPressed = 0;
}

void Scene::onMouseMotion(int mouseX, int mouseY) {
    // this function is called every time the mouse moves while a button is pressed


    double x, y, z;
    converter.convert(mouseX, mouseY, x, y, z);

    double dX = x - mouseClickStartX;
    double dY = y - mouseClickStartY;

    double eX = x - mousePreviousX;
    double eY = y - mousePreviousY;

    printf("Scene::onMouseMotion called with x=%f, y=%f    dX=%f, dY=%f\n", x, y, dX, dY);
    if(mouseButtonPressed == GLUT_LEFT_BUTTON){
        // left button is translation
        translateX += eX;
        translateY += eY;
    } else if (mouseButtonPressed == GLUT_RIGHT_BUTTON) {
        // right button is rotation
    }
    mousePreviousX = x;
    mousePreviousY = y;
}

MouseToWorldConverter::MouseToWorldConverter(){
    glGetDoublev(GL_MODELVIEW_MATRIX, modelViewMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
    glGetIntegerv(GL_VIEWPORT, viewport);
}

void MouseToWorldConverter::convert(int mouseX, int mouseY, double& objX, double& objY, double& objZ) {
    gluUnProject(mouseX, mouseY, 0,
                 modelViewMatrix, projectionMatrix, viewport,
                 &objX, &objY, &objZ);
}

