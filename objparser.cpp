#include "objparser.h"

using namespace Eigen;

ParsedObj::ParsedObj(){
    scale = 1.0;
}

void ParsedObj::centerAndScale(float s){
    /* centers the obj's vertices about the origin
     * and then scales them so that the furthest vertex
     * has a distance of 's' from the origin */
    if(vertices.size() < 1){
        printf("Invalid number of vertices: %i\n", int(vertices.size()));
        return;
    }

    Eigen::Vector3f minVertex = *vertices[0];
    Eigen::Vector3f maxVertex = *vertices[0];
    Eigen::Vector3f* iterVertex;

    for(unsigned int i=0; i<vertices.size(); i++){
        iterVertex = vertices[i];
        if(iterVertex->x() < minVertex.x()){
            minVertex.x() = iterVertex->x();
        }
        if(iterVertex->y() < minVertex.y()){
            minVertex.y() = iterVertex->y();
        }
        if(iterVertex->z() < minVertex.z()){
            minVertex.z() = iterVertex->z();
        }
        if(iterVertex->x() > maxVertex.x()){
            maxVertex.x() = iterVertex->x();
        }
        if(iterVertex->y() > maxVertex.y()){
            maxVertex.y() = iterVertex->y();
        }
        if(iterVertex->z() > maxVertex.z()){
            maxVertex.z() = iterVertex->z();
        }
    }
    center = (minVertex + maxVertex) * 0.5;
    Eigen::Vector3f size = maxVertex - minVertex;

    sizeMultiplier = 1.0f / size.maxCoeff();
    scale = sizeMultiplier * s * 2;

    for(unsigned int i=0; i<vertices.size(); i++){
        iterVertex = vertices[i];

        *iterVertex -= center;
        *iterVertex *= scale;
    }

    printf("Center found at: %f %f %f\n", center.x(), center.y(), center.z());
    printf("sizeMultiplier: %f\n", sizeMultiplier);
    printf("Scaling by: %f\n", scale);
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
    std::ifstream objFile (filename.c_str());

    std::string op;
    float x, y, z;
    int vert1, vert2, vert3; // for faces, the indexes of the verts

    if(objFile.is_open()){
        while(getline(objFile, line)){
            if(line.empty()){
                continue;
            }
            std::stringstream lineStream;
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
                obj->faces.push_back(face);
            }

        }
    } else {
        printf("Unable to open file '%s'\n", filename.c_str());
    }

    return obj;
}
