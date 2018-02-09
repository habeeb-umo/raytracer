//
// Created by Habeeb Mohammed on 10/12/17.
//

#ifndef CS410_OBJECT_H
#define CS410_OBJECT_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "triangle.h"

class Model{

public:

    //transformation elements
    Eigen::Vector4d rotAxis;
    Eigen::Vector4d translation;
    Eigen::MatrixXd transformed;
    double angle;
    double scale;
    std::string filename;
    std::string mtlFile;

    //.obj elements
    std::vector<std::string> comments;
    Eigen::MatrixXd vertices;
    std::vector<Eigen::Vector3d> faceRefs;
    Eigen::MatrixXd vertexNorms;
    std::vector<Triangle> faces;

    //todo:remember to include mtl eventually
    Eigen::Vector3d ka, kd, ks;
    double phong;

};

#endif //CS410_OBJECT_H
