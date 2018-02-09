//
// Created by Habeeb Mohammed on 10/12/17.
//

#ifndef CS410_LIGHT_H
#define CS410_LIGHT_H

#include <string>
#include <vector>
#include <Eigen/Dense>

class Light{

public:

    Eigen::Vector3d position;
    int w;
    Eigen::Vector3d emission;

};

#endif //CS410_LIGHT_H
