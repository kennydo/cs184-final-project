#ifndef SKELETON_H
#define SKELETON_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include "joint.h"
#include <set>
#include <string>
#include <vector>

class Skeleton {
    public:
        Skeleton();
        static Skeleton* parse(std::string);

        std::set<int> endEffectorIds;
        std::vector<Link*> joints;
        std::vector<Link*> parents;

        void offset(Eigen::Vector3f);
        void scale(float);
};

#endif
