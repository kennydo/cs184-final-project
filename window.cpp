#include "window.h"
#include <cstdio>

int Window::width = 0;
int Window::height = 0;
std::string Window::title = "";
Scene* Window::scene;

void Window::init(int w, int h, std::string t, int pos_x, int pos_y, Scene* s){
    Window::width = w;
    Window::height = h;
    Window::title = t;
    Window::scene = s;

    //This tells glut to use a double-buffered window with RGB channels
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    // creates the window by calling the necessary glut stuff
    glutInitWindowSize(Window::width, Window::height);
    glutInitWindowPosition(pos_x, pos_y);
    glutCreateWindow(Window::title.c_str());

    // clear scene to black, fully transparent
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    glEnable(GL_DEPTH_TEST);

    // re-normalize things automatically because scaling messes with normals
    glEnable(GL_NORMALIZE);

    // by default, we are in wireframe mode, so no need for lighting

    glutDisplayFunc(Window::display);
    glutReshapeFunc(Window::reshape);
    glutKeyboardFunc(Window::keyboard);
    glutSpecialFunc(Window::specialKeys);
    glutMouseFunc(Window::mouse);
    glutIdleFunc(Window::idle);
}

void Window::display(){
    // set everything to black
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // this is where the magic happens
    // it's time to draw polygons and stuff!

    Window::scene->refreshCamera(0, 0);
    Window::scene->draw();

    glFlush();
    glutSwapBuffers();
}

void Window::reshape(int w, int h){
    Window::width = w;
    Window::height = h;
    glViewport(0, 0, Window::width, Window::height);
}

void Window::idle(){
    // if we are on WIN32, sleep for 10 ms
    // but we're not, so we don't care

    glutPostRedisplay();
}

void Window::keyboard(unsigned char key, int x, int y) {
    switch(key) {
        case '+':
            printf("rotate joint clockwise\n");
            scene->rotateSkeleton(30);
            break;
        case '-':
            printf("rotate joint counterclockwise\n");
            scene->rotateSkeleton(-30);
            break;
        case ' ':
            printf("quitting\n");
            exit(0);
            break;
    }
}

void Window::specialKeys(int key, int x, int y) {
    switch(key) {
        case GLUT_KEY_UP:
            printf("moving joint up\n");
            scene->moveSkeleton(10);
            break;
        case GLUT_KEY_DOWN:
            printf("moving joint down\n");
            scene->moveSkeleton(-10);
            break;
    }
}

void Window::mouse(int button, int state, int x, int y) {
    int window_height = glutGet(GLUT_WINDOW_HEIGHT);
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        // opengl expects y=0 to be the bottom of the screen,
        // but glut has y=- be the top of the screen, so we do math
        scene->getNumClickHits(x, window_height - y);
    }
}
