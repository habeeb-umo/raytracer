//
// Created by Habeeb Mohammed on 10/12/17.
//
#ifndef CS410_CAMERA_H
#define CS410_CAMERA_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "ray.h"

class Camera{

public:

    Eigen::Vector3d eyePt, lookAtVec, upVec, wVec, uVec, vVec;
    double distance;
    std::vector<double> bounds;
    std::vector<double> resolution;
    std::vector<std::vector<Ray> > rays;

};
#endif //CS410_CAMERA_H
