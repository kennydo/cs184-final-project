#include "window.h"

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
    glutIdleFunc(Window::idle);
}

void Window::display(){
    // set everything to black
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // this is where the magic happens
    // it's time to draw polygons and stuff!

    Window::scene->refreshCamera();
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
