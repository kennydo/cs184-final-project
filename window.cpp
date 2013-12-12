#include "window.h"
#include <cstdio>
#include <Eigen/Geometry>

const double PI = 4.0*atan(1.0);

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
    glutMotionFunc(Window::motion);
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
        case 'w':
        {
            printf("rotate joint clockwise around x axis\n");
            Quaternionf q(-sqrt(0.5), sqrt(0.5), 0, 0);
            scene->rotateTestSkeleton(q);
        }
            break;
        case 's':
        {
            printf("rotate joint counterclockwise around x axis\n");
            Quaternionf q(sqrt(0.5), sqrt(0.5), 0, 0);
            scene->rotateTestSkeleton(q);

        }
            break;
        case 'a':
        {
            printf("rotate joint clockwise around z axis\n");
            Quaternionf q(-sqrt(0.5), 0, 0, sqrt(0.5));
            scene->rotateTestSkeleton(q);
        }
            break;
        case 'd':
        {
            printf("rotate joint counterclockwise around z axis\n");

            Quaternionf q(sqrt(0.5), 0, 0, sqrt(0.5));
            scene->rotateTestSkeleton(q);
        } 
            break;
        case 'z':
        {
            printf("moving joint forward\n");
            scene->moveTestSkeleton(0.0f, 0.0f, 1.0f);
        }
            break;
        case 'x':
        {
            printf("moving joint backward\n");
            scene->moveTestSkeleton(0.0f, 0.0f, -1.0f);
        }
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
            scene->moveTestSkeleton(0.0f, 1.0f, 0.0f);
            break;
        case GLUT_KEY_DOWN:
            printf("moving joint down\n");
            scene->moveTestSkeleton(0.0f, -1.0f, 0.0f);
            break;
        case GLUT_KEY_LEFT:
            printf("moving joint left\n");
            scene->moveTestSkeleton(-1.0f, 0.0f, 0.0f);
            break;
        case GLUT_KEY_RIGHT:
            printf("moving joint right\n");
            scene->moveTestSkeleton(1.0f, 0.0f, 0.0f);
            break;
    }
}

void Window::mouse(int button, int state, int x, int y) {
    int windowHeight = glutGet(GLUT_WINDOW_HEIGHT);

    // opengl expects y=0 to be the bottom of the screen,
    // but glut has y=- be the top of the screen, so be sure to do
    // math. Pass in (x, windowHeight - y)
    if(button == GLUT_LEFT_BUTTON) {
        if(state == GLUT_DOWN){
            scene->onLeftClick(x, windowHeight - y);
        } else if(state == GLUT_UP){
            scene->onLeftRelease(x, windowHeight - y);
        }
    } else if (button == GLUT_RIGHT_BUTTON){
        if(state == GLUT_DOWN){
            scene->onRightClick(x, windowHeight - y);
        } else if(state == GLUT_UP){
            scene->onRightRelease(x, windowHeight - y);
        }
    } else if (button == 3) {
        // stackoverflow ( http://stackoverflow.com/questions/14378/using-the-mouse-scrollwheel-in-glut )
        // said that 3 was scroll up
        scene->onZoomIn();
    } else if (button == 4) {
        // the same stackoverflow said that 4 was scroll down
        scene->onZoomOut();
    }
}

void Window::motion(int x, int y) {
    int windowHeight = glutGet(GLUT_WINDOW_HEIGHT);

    scene->onMouseMotion(x, windowHeight - y);
}
