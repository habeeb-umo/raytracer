//
// Created by habeeb on 10/26/17.
//

#ifndef CS410_RAY_H
#define CS410_RAY_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "triangle.h"
#include "sphere.h"

class Ray{

public:

    Eigen::Vector3d direction;
    Eigen::Vector3d pixPt;

    Triangle intersect;
    Eigen::Vector3d intersectPt;
    Eigen::Vector3d color;

    double bestT;
    Sphere bestSphere;
    Triangle bestTriangle;
    Eigen::Vector3d bestPt;

    bool hitSphere;
    bool hitTriangle;
    bool refracted;

    double beta;
    double gamma;


};

#endif //CS410_RAY_H
