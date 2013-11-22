#ifndef SCENE_H
#define SCENE_H

#include <GL/glu.h>
#include <GL/glut.h>

class Scene {
    public:
        Scene();

        void refreshCamera();
        void draw();

        void drawGrid(float, float, float, float, float);
};

#endif
