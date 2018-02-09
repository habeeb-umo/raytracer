//
// Created by Habeeb Mohammed on 10/12/17.
//
#ifndef CS410_TRIANGLE_H
#define CS410_TRIANGLE_H

#include <string>
#include <vector>
#include <Eigen/Dense>

class Triangle{

public:

    Eigen::Vector3d vertA;
    Eigen::Vector3d vertB;
    Eigen::Vector3d vertC;

    Eigen::Vector3d surfaceNorm;

};

#endif //CS410_TRIANGLE_H
