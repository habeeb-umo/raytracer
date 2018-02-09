//
// Created by Habeeb Mohammed on 10/12/17.
//

#ifndef CS410_SCENE_H
#define CS410_SCENE_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "driver.h"
#include "camera.h"
#include "model.h"
#include "triangle.h"
#include "light.h"

class Scene{

public:
    Camera camera;
    std::vector<Model> models;
    std::vector<Sphere> spheres;
    std::vector<Light> lights;
    Eigen::Vector3d ambiance;
    int recursionLevel;
    double eta_outside;

    std::vector<std::vector<Eigen::Vector3d> > image;


};
#endif //CS410_SCENE_H
