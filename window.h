#ifndef WINDOW_H
#define WINDOW_H

#include <GL/glut.h>
#include <GL/glu.h>
#include <string>

class Window {
    public:
        static int width, height;
        static std::string title;
        static void init(int, int, std::string, int, int);
        static void reshape(int, int);
        static void idle();
        static void display();
};

#endif
