//
// Created by Habeeb Mohammed on 10/12/17.
//

#ifndef CS410_SPHERE_H
#define CS410_SPHERE_H

#include <string>
#include <vector>
#include <Eigen/Dense>

class Sphere{

    public:
        Eigen::Vector3d center;
        double radius;
        Eigen::Vector3d ambient;
        Eigen::Vector3d diffuse;
        Eigen::Vector3d specular;
        Eigen::Vector3d attenuation;
        Eigen::Vector3d opacity;
        double phong;
        double eta;
        bool isReal;

        std::vector<Eigen::Vector3d> intersectedCoords;

};

#endif //CS410_SPHERE_H
