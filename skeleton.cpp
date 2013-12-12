#include "skeleton.h"

Skeleton::Skeleton(){

}

Skeleton* Skeleton::parse(std::string filename){
    Skeleton* skeleton = new Skeleton();

    std::string line;
    std::stringstream lineStream;
    std::ifstream skeletonFile (filename.c_str());
    Eigen::Vector3f position, parentPosition;

    // when a joint has parent -1, it's the root joint
    signed int id, parent;
    float x, y, z;
    float length;


    Eigen::Quaternionf identityQuaternion(1, 0, 0, 0);

    if(skeletonFile.is_open()){
        while(getline(skeletonFile, line)){
            if(line.empty()){
                continue;
            }

            lineStream << line;
            lineStream >> id >> x >> y >> z >> parent;

            assert(id == int(skeleton->joints.size()));

            position = Eigen::Vector3f(x, y, z);
            if(parent >= 0){
                parentPosition = skeleton->joints[parent]->pos();
                length = std::sqrt(
                            std::pow(parentPosition.x() - position.x(), 2) +
                            std::pow(parentPosition.y() - position.y(), 2) +
                            std::pow(parentPosition.z() - position.z(), 2)
                        );
            } else {
                length = 0;
            }
            Link* joint = new Link(length, position, identityQuaternion);
            skeleton->joints.push_back(joint);
            if(parent >= 0){
                // add the inner and outer links
                skeleton->joints[parent]->addOuterLink(id);
                joint->addInnerLink(parent);
            }
        }
    } else {
        printf("Unable to open file '%s'\n", filename.c_str());
    }

    return skeleton;
}
