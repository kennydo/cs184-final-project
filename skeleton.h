#ifndef SKELETON_H
#define SKELETON_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include "joint.h"
#include <string>
#include <vector>

class Skeleton {
    public:
        Skeleton();
        static Skeleton* parse(std::string);

        std::vector<Link*> joints;
        std::vector<Link*> parents;
};

#endif
