#include "objparser.h"

using namespace Eigen;

ParsedObj::ParsedObj(){
    // nothing for now
}

ObjFace::ObjFace(Eigen::Vector3f* vert1, Eigen::Vector3f* vert2, Eigen::Vector3f* vert3){
    vertices[0] = vert1;
    vertices[1] = vert2;
    vertices[2] = vert3;

    // vertices are defined counter-clockwise?
    Eigen::Vector3f edge1 = *vert1 - *vert2;
    Eigen::Vector3f edge2 = *vert2 - *vert3;
    normal = new Eigen::Vector3f(edge2.cross(edge1));
    normal->normalize();
}

ParsedObj* ObjParser::parse(std::string filename){

    ParsedObj* obj = new ParsedObj();

    std::string line;
    std::stringstream lineStream;
    std::ifstream objFile (filename.c_str());

    std::string op;
    float x, y, z;
    int vert1, vert2, vert3; // for faces, the indexes of the verts

    if(objFile.is_open()){
        while(getline(objFile, line)){
            if(line.empty()){
                continue;
            }

            lineStream << line;

            lineStream >> op;
            if(op.empty() || op[0] == '#'){
                continue;
            }

            if(op == "v"){
                // it's a vertex
                lineStream >> x >> y >> z;

                Eigen::Vector3f* vertex = new Eigen::Vector3f(x, y, z);
                obj->vertices.push_back(vertex);
            } else if (op == "f"){
                // it's a face
                lineStream >> vert1 >> vert2 >> vert3;
                ObjFace* face = new ObjFace(obj->vertices[vert1 - 1],
                                            obj->vertices[vert2 - 1],
                                            obj->vertices[vert3 - 1]);
                printf("%f\n", face->normal->x());
                obj->faces.push_back(face);
            }

        }
    } else {
        printf("Unable to open file '%s'\n", filename.c_str());
    }

    return obj;
}
