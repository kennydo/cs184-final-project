#ifndef OBJPARSER_H
#define OBJPARSER_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class ObjFace {
    public:
        ObjFace(Eigen::Vector3f*, Eigen::Vector3f*, Eigen::Vector3f*);
        Eigen::Vector3f* vertices[3];
        Eigen::Vector3f* normal;
};

class ParsedObj {
    public:
        ParsedObj();
        std::vector<Eigen::Vector3f*> vertices;
        std::vector<ObjFace*> faces;

        Eigen::Vector3f center;
        float sizeMultiplier;
        float scale;

        void centerAndScale(float);
};

class ObjParser {
    public:
        static ParsedObj* parse(std::string);
};

#endif
