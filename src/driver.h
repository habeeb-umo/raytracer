//
// Created by Habeeb Mohammed on 9/15/17.
//
#include <string>
#include <vector>
#include <Eigen/Dense>

#ifndef CS410_DRIVER_H
#define CS410_DRIVER_H
class Driver {

    public:
        std::string keyword;

        //model stuff
        Eigen::Vector4d rotAxis;
        double rotAngle;
        double scale;
        Eigen::Vector4d translate;
        std::string filename;
        std::string folder;

        //camera stuff


        //optional sphere
        std::vector<double> sphere;

};
#endif //CS410_DRIVER_H
