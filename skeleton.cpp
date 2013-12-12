#include "skeleton.h"

Skeleton::Skeleton(){
}

Skeleton* Skeleton::parse(std::string filename){
    Skeleton* skeleton = new Skeleton();

    std::string line;
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
            std::stringstream lineStream;

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
            skeleton->endEffectorIds.insert(id);
            if(parent >= 0){
                skeleton->endEffectorIds.erase(parent);
                // add the inner and outer links
                skeleton->joints[parent]->addOuterLink(id);
                joint->addInnerLink(parent);
                skeleton->parents.push_back(skeleton->joints[parent]);
            } else {
                skeleton->parents.push_back(NULL);
            }
        }
    } else {
        printf("Unable to open file '%s'\n", filename.c_str());
    }
    printf("Got the following endEffectorIds:\n");
    for(std::set<int>::iterator it=skeleton->endEffectorIds.begin(); it!=skeleton->endEffectorIds.end(); ++it){
        int id = *it;
        printf("\t%d\n", id);
    }
    return skeleton;
}

void Skeleton::offset(Eigen::Vector3f offsetAmount){
    Link* joint;
    Eigen::Vector3f currentPosition;
    for(unsigned int i=0; i < joints.size() ; i++){
        joint = joints[i];

        currentPosition = joint->pos();
        joint->moveJoint(currentPosition + offsetAmount);
    }
}

void Skeleton::scale(float scale){
    Link* joint;
    Eigen::Vector3f currentPosition;
    float currentLength;
    for(unsigned int i=0; i < joints.size(); i++){
        joint = joints[i];
        currentPosition = joint->pos();
        currentLength = joint->getLength();

        joint->moveJoint(currentPosition * scale);
        joint->setLength(currentLength * scale);
    }
}

bool Skeleton::isEndEffector(int id){
    return endEffectorIds.find(id) != endEffectorIds.end();
}
