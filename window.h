#ifndef WINDOW_H
#define WINDOW_H

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <string>
#include "scene.h"

class Window {
    public:
        static Scene* scene;
        static int width, height;
        static std::string title;
        static void init(int, int, std::string, int, int, Scene*);
        static void reshape(int, int);
        static void idle();
        static void display();
        static void keyboard(unsigned char, int, int);
        static void specialKeys(int, int, int);
        static void mouse(int, int, int, int);
};

#endif
