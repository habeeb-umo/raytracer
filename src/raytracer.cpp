//
// Created by Habeeb Mohammed on 10/12/17.
//
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <stdlib.h>
#include <cmath>
#include <string>
#include <fstream>
#include <regex>
#include <vector>
#include <map>
#include <tuple>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include "driver.h"
#include "camera.h"
#include "model.h"
#include "triangle.h"
#include "light.h"
#include "sphere.h"
#include "scene.h"
#include "ray.h"


//todo: implement rayTriangleRGB
//todo: handle multiple objects in scene

#define INF 99999

using namespace std;

//Parsing
void parseDriver(Scene &s, vector<string> &v);
void parseObjs(vector<Model> &mv);
void parseMtls(vector<Model> &mv);

//Transformation
void transformObjs(vector<Model> &mv);
Model translate(Model &m);
Model scale(Model &m);
Model rotate(Model &m);

//Scene setup
void placeCamera(Camera &c);
Ray pixelRay(int i, int j, Camera c);

//Intersection
bool raySphere(Ray &r, Sphere s);
Sphere rayFindSphere(Ray &r, vector<Sphere> s);
Triangle rayFindModel(Ray &r, vector<Model> mv);

//Illumination
Eigen::Vector3d rayTrace(Ray &r, Eigen::Vector3d ac, Eigen::Vector3d at, int lvl, vector<Sphere> vs, Scene &s);
bool shadow(Eigen::Vector3d pt, Light light, vector<Sphere> spheres);
Eigen::Vector3d refractTray(Ray ray, Eigen::Vector3d W, Eigen::Vector3d pt, Eigen::Vector3d N, double eta1, double eta2);
Ray refractExit(Ray ray, Eigen::Vector3d W, Eigen::Vector3d pt, double eta_in, double eta_out);
//Unit tests
void testScene(Scene &s);

//Output
void writeOut(Scene &s, string &str);


int main(int argc, char* argv[]){

    if(argc != 3){
        cerr << "Not enough arguments provided" << '\n' <<
             "Usage: ./raytracer [driver.txt] [destination.ppm]" << '\n';
        exit(0);
    }

    ifstream driverFile;
    vector<string> driverLines;

    //Grab lines from driver.txt
    if(argv[1]){
        string driverLine;
        driverFile.open(argv[1]);
        while(getline(driverFile,driverLine)){
            driverLines.push_back(driverLine);
        }
        driverFile.close();
    }

    //Create Driver object for driver file
    //Driver driver;
    //Create all elements possibly referenced in Scene
    Scene scene;
    vector<Model> totalModels;
    vector<Sphere> totalSpheres;
    vector<Light> totalLights;

    Camera camera;

    scene.models = totalModels;
    scene.spheres = totalSpheres;
    scene.lights = totalLights;
    scene.camera = camera;


    vector<string> parse;
    for(size_t i = 0; i < driverLines.size(); i++) {
        string buffer;
        stringstream ss(driverLines[i]);
        while (ss >> buffer) {
            parse.push_back(buffer);
        }
    }

    //Parse all values in driver file into respective objects (by reference of course!)
    //Also populate Scene object with those elements
    parseDriver(scene, parse);

    if(!scene.models.empty()) {
        //parse all model objs
        parseObjs(scene.models);
        //parse all obj mtls
        parseMtls(scene.models);
        //transform all model objs
        transformObjs(scene.models);
    }

    //place camera in scene
    placeCamera(scene.camera);
    //place pixels in scene & throw rays
    vector< vector<Ray> > rays;
    for(int i = 0; i < scene.camera.resolution[0]; i++){
        vector<Ray> rayRow;
        for(int j = 0; j < scene.camera.resolution[1]; j++){
            Ray ray;
            ray = pixelRay(i, j, scene.camera);
            rayRow.push_back(ray);
        }
        rays.push_back(rayRow);
    }

    scene.camera.rays = rays;

    for(size_t i = 0; i < rays.size(); i++){
        for(size_t j = 0; j < rays[i].size(); j++) {
            Eigen::Vector3d atten, accum, color;
            color << 0,0,0;
            accum << 0,0,0;
            atten << .8,.8,.8;

            color = rayTrace(rays[i][j], accum, atten, scene.recursionLevel, scene.spheres, scene);
            //cout << "COLOR " << color << '\n';
            color(0) = int(color(0) * 255);
            if(color(0) > 255){
                color(0) = 255;
            }
            color(1) = int(color(1) * 255);
            if(color(1) > 255){
                color(1) = 255;
            }
            color(2) = int(color(2) * 255);
            if(color(2) > 255){
                color(2) = 255;
            }
            scene.image[i][j] = color;
        }
    }

    string filename = argv[2];
    writeOut(scene, filename);


    testScene(scene);



}

void testScene(Scene &scene){

    cout << '\n';
    cout << "__________UNIT_TESTS___________"<< '\n';
    cout << '\n';

    cout << "Camera elements: " << '\n';
    cout << "    resolution: " << scene.camera.resolution[0] << ", " << scene.camera.resolution[1] << '\n';
    cout << "    bounds: " << scene.camera.bounds[0] << ", " << scene.camera.bounds[1]
         << ", " << scene.camera.bounds[2] << ", " << scene.camera.bounds[3] << '\n';
    cout << "    distance: " << scene.camera.distance << '\n';
    cout << "    up vector: " << scene.camera.upVec(0) << ", " << scene.camera.upVec(1)
         << ", " << scene.camera.upVec(2) << '\n';
    cout << "    look at point: " << scene.camera.lookAtVec(0) << ", " << scene.camera.lookAtVec(1)
         << ", " << scene.camera.lookAtVec(2) << '\n';
    cout << "    eye point: " << scene.camera.eyePt(0) << ", " << scene.camera.eyePt(1)
         << ", " << scene.camera.eyePt(2) << '\n';
    cout << "    u vector: " << scene.camera.uVec(0) << ", " << scene.camera.uVec(1)
         << ", " << scene.camera.uVec(2) << '\n';
    cout << "    v vector: " << scene.camera.vVec(0) << ", " << scene.camera.vVec(1)
         << ", " << scene.camera.vVec(2) << '\n';
    cout << "    w vector: " << scene.camera.wVec(0) << ", " << scene.camera.wVec(1)
         << ", " << scene.camera.wVec(2) << '\n';
    cout << "    rays.direction(0,0): "
         << scene.camera.rays[0][0].direction(0) << ", "
         << scene.camera.rays[0][0].direction(1) << ", "
         << scene.camera.rays[0][0].direction(2) << '\n';
    cout << "    rays.direction(last,last): "
         << scene.camera.rays[scene.camera.rays.size()-1][scene.camera.rays.size()-1].direction(0) << ", "
         << scene.camera.rays[scene.camera.rays.size()-1][scene.camera.rays.size()-1].direction(1) << ", "
         << scene.camera.rays[scene.camera.rays.size()-1][scene.camera.rays.size()-1].direction(2) << '\n';
    cout << "    rays.pixPt(0,0): "
         << scene.camera.rays[0][0].pixPt(0) << ", "
         << scene.camera.rays[0][0].pixPt(1) << ", "
         << scene.camera.rays[0][0].pixPt(2) << '\n';
    cout << "    rays.pixPt(last,last): "
         << scene.camera.rays[scene.camera.rays.size()-1][scene.camera.rays.size()-1].pixPt(0) << ", "
         << scene.camera.rays[scene.camera.rays.size()-1][scene.camera.rays.size()-1].pixPt(1) << ", "
         << scene.camera.rays[scene.camera.rays.size()-1][scene.camera.rays.size()-1].pixPt(2) << '\n';
    cout << '\n';

    cout << "Sphere elements: " << '\n';
    cout << "    number of spheres: " << scene.spheres.size() << '\n';
    cout << "    spheres[0].center: "
         << scene.spheres[0].center(0) << ", "
         << scene.spheres[0].center(1) << ", " << scene.spheres[0].center(2) << '\n';
    cout << "    spheres[last].center: "
         << scene.spheres[scene.spheres.size()-1].center(0) << ", "
         << scene.spheres[scene.spheres.size()-1].center(1) << ", "
         << scene.spheres[scene.spheres.size()-1].center(2) << '\n';
    cout << "    sphere[0] radius: " << scene.spheres[0].radius << '\n';
    cout << "    sphere[last] radius: " << scene.spheres[scene.spheres.size()-1].radius << '\n';
    cout << "    spheres[0].ambient: "
         << scene.spheres[0].ambient(0) << ", "
         << scene.spheres[0].ambient(1) << ", " << scene.spheres[0].ambient(2) << '\n';
    cout << "    spheres[last].ambient: "
         << scene.spheres[scene.spheres.size()-1].ambient(0) << ", "
         << scene.spheres[scene.spheres.size()-1].ambient(1) << ", "
         << scene.spheres[scene.spheres.size()-1].ambient(2) << '\n';
    cout << "    spheres[0].diffuse: "
         << scene.spheres[0].diffuse(0) << ", "
         << scene.spheres[0].diffuse(1) << ", " << scene.spheres[0].diffuse(2) << '\n';
    cout << "    spheres[last].diffuse: "
         << scene.spheres[scene.spheres.size()-1].diffuse(0) << ", "
         << scene.spheres[scene.spheres.size()-1].diffuse(1) << ", "
         << scene.spheres[scene.spheres.size()-1].diffuse(2) << '\n';
    cout << "    spheres[0].specular: "
         << scene.spheres[0].specular(0) << ", "
         << scene.spheres[0].specular(1) << ", " << scene.spheres[0].specular(2) << '\n';
    cout << "    spheres[last].specular: "
         << scene.spheres[scene.spheres.size()-1].specular(0) << ", "
         << scene.spheres[scene.spheres.size()-1].specular(1) << ", "
         << scene.spheres[scene.spheres.size()-1].specular(2) << '\n';
    cout << "    spheres[0].attenuation: "
         << scene.spheres[0].attenuation(0) << ", "
         << scene.spheres[0].attenuation(1) << ", " << scene.spheres[0].attenuation(2) << '\n';
    cout << "    spheres[last].attenuation: "
         << scene.spheres[scene.spheres.size()-1].attenuation(0) << ", "
         << scene.spheres[scene.spheres.size()-1].attenuation(1) << ", "
         << scene.spheres[scene.spheres.size()-1].attenuation(2) << '\n';
    cout << '\n';

    cout << "Light elements: " << '\n';
    cout << "    number of lights: " << scene.lights.size() << '\n';
    cout << "    light[0].w: " << scene.lights[0].w << '\n';
    cout << "    light[last].w: " << scene.lights[scene.lights.size()-1].w << '\n';
    cout << "    light[0].position: " << scene.lights[0].position(0) << ", "
         << scene.lights[0].position(1) << ", " << scene.lights[0].position(2) << '\n';
    cout << "    light[last].position: " << scene.lights[scene.lights.size()-1].position(0) << ", "
         << scene.lights[scene.lights.size()-1].position(1) << ", "
         << scene.lights[scene.lights.size()-1].position(2) << '\n';
    cout << "    light[0].emission: " << scene.lights[0].emission(0) << ", "
         << scene.lights[0].emission(1) << ", " << scene.lights[0].emission(2) << '\n';
    cout << "    light[last].emission: " << scene.lights[scene.lights.size()-1].emission(0) << ", "
         << scene.lights[scene.lights.size()-1].emission(1) << ", "
         << scene.lights[scene.lights.size()-1].emission(2) << '\n';
    cout << '\n';

    cout << "Recursion Level: " << scene.recursionLevel << '\n';

}

void writeOut(Scene &scene, string &file){

    ofstream outfile(file);
    if(outfile){
        outfile << "P3" << '\n';
        outfile << scene.camera.resolution[0] << " " << scene.camera.resolution[1] << " " << "255" << '\n';
        for(size_t i = 0; i < scene.image.size(); i++){
            for(size_t j = 0; j < scene.image[i].size(); j++){
                outfile << scene.image[i][j](0) << " " << scene.image[i][j](1) << " " << scene.image[i][j](2) << " ";
            }
            outfile << '\n';
        }

    }
    outfile.close();
}

Eigen::Vector3d rayTrace(Ray &ray, Eigen::Vector3d accum, Eigen::Vector3d atten,
                         int level, vector<Sphere> spheres, Scene &scene){

    Sphere sfound = rayFindSphere(ray, spheres);
    ray.bestSphere = sfound;

    if(sfound.isReal && ray.hitSphere) {
        Eigen::Vector3d snrm, color, toC;


        snrm = (ray.bestPt - ray.bestSphere.center).normalized();
        scene.ambiance = scene.ambiance * .7;
        color << scene.ambiance(0) * ray.bestSphere.ambient(0), scene.ambiance(1) * ray.bestSphere.ambient(1),
                scene.ambiance(2) * ray.bestSphere.ambient(2);

        toC = -1 * ray.direction.normalized();

        if(snrm.dot(toC) < 0){
            snrm = snrm * -1;
        }

        for(auto light : scene.lights){
            Eigen::Vector3d toL;

            toL = (light.position - ray.bestPt).normalized();

            if(snrm.dot(toL) > 0.0 && (!shadow(ray.bestPt,light,spheres))){
                Eigen::Vector3d kd, ks, spR;

                    kd << ray.bestSphere.diffuse(0) * light.emission(0), ray.bestSphere.diffuse(1) *
                            light.emission(1), ray.bestSphere.diffuse(2) * light.emission(2);
                    color += (kd * snrm.dot(toL));

                    spR = ((2 * snrm.dot(toL) * snrm) - toL).normalized();

                    if(toC.dot(spR) > 0){
                        ks << ray.bestSphere.specular(0) * light.emission(0), ray.bestSphere.specular(1) * light.emission(1),
                                ray.bestSphere.specular(2) * light.emission(2);
                        //cout << kd * pow(toC.dot(spR), 16) << '\n';
                        color += (ks * pow(toC.dot(spR), ray.bestSphere.phong));
                    }
            }
        }

        ray.bestSphere.attenuation = ray.bestSphere.attenuation * .8;
        for(int i = 0; i < 3; i++){
            accum(i) += atten(i) * ray.bestSphere.opacity(i) * color(i);
        }


        if(level > 0){
            Eigen::Vector3d refR, flec, uInv, N;
            N = (ray.bestPt - ray.bestSphere.center).normalized();
            uInv = -1 * ray.direction;
            flec << 0,0,0;
            refR = ((2 * N.dot(uInv) * N) - uInv).normalized();

            Ray recRay = ray;
            recRay.pixPt = ray.bestPt;
            recRay.direction = refR;

            atten(0) = atten(0) * ray.bestSphere.attenuation(0);
            atten(1) = atten(1) * ray.bestSphere.attenuation(1);
            atten(2) = atten(2) * ray.bestSphere.attenuation(2);

            flec = rayTrace(recRay, flec, atten, (level - 1), spheres, scene);
            for(int i = 0; i < 3; i++){
                accum(i) += (atten(i) * ray.bestSphere.opacity(i) * flec(i));
            }

            double opSum = ray.bestSphere.opacity(0) + ray.bestSphere.opacity(1) + ray.bestSphere.opacity(2);
            if((level > 0) && (opSum < 3.0)){
                Eigen::Vector3d thru;
                Ray fraR;
                thru << 0,0,0;
                ray.direction = ray.direction * -1;
                fraR = refractExit(ray, -1*ray.direction, ray.bestPt, ray.bestSphere.eta, scene.eta_outside);
                if(fraR.refracted){
                    Eigen::Vector3d attens;
                    for(int i = 0; i < 3; i++){
                        attens(i) = ray.bestSphere.attenuation(i) * atten(i);
                    }
                    thru = rayTrace(fraR, thru, attens, (level -1), spheres, scene);
                    for(int i = 0; i < 3; i++){
                        accum(i) += (atten(i) * (1.0 - ray.bestSphere.opacity(i)) * thru(i));
                    }
                }
            }
        }

    }

    return accum;
}

Eigen::Vector3d refractTray(Eigen::Vector3d W, Eigen::Vector3d pt, Eigen::Vector3d N, double eta1, double eta2){
    double etar = eta1 / eta2;
    double a = -1 * etar;
    double wn = W.dot(N);
    double radsq = pow(etar, 2) * (pow(wn, 2) - 1) + 1;

    Eigen::Vector3d T;
    if(radsq < 0.0){
        T << 0,0,0;
    }
    else{
        double b = (etar * wn) - sqrt(radsq);
        Eigen::Vector3d aW = a * W;
        Eigen::Vector3d bN = b * N;
        T = aW + bN;
    }

    return T;
}

Ray refractExit(Ray ray, Eigen::Vector3d W, Eigen::Vector3d pt, double eta_in, double eta_out){
    Eigen::Vector3d ptMinC = (pt - ray.bestSphere.center).normalized();
    Eigen::Vector3d T1 = refractTray(W, pt, ptMinC, eta_out, eta_in);
    double T1Sum = T1(0) + T1(1) + T1(2);

    if(T1Sum != 0){
        double cDotT1 = (ray.bestSphere.center - pt).dot(T1);
        Eigen::Vector3d exit = pt + 2 * cDotT1 * T1;
        Eigen::Vector3d Nin = (ray.bestSphere.center - exit).normalized();
        Eigen::Vector3d T2 = refractTray(-T1, exit, Nin, eta_in, eta_out);
        Ray refR = ray;
        refR.pixPt = exit;
        refR.direction = T2;
        refR.refracted = true;
        return refR;
    }
    else{
        ray.refracted = false;
        return ray;
    }
}

bool shadow(Eigen::Vector3d pt, Light light, vector<Sphere> spheres){
    Eigen::Vector3d ptDiff = light.position - pt;
    Ray ray;
    ray.pixPt = pt;
    ray.direction = ptDiff.normalized();
    ray.bestT = INF;

    double dtl = ptDiff.dot(ray.direction);
    for(auto s : spheres){
        if(raySphere(ray,s) && ray.bestT < dtl){
            return true;
        }
    }
    return false;
}

bool raySphere(Ray &ray, Sphere sphere){

    Eigen::Vector3d tVec, pt;
    double v,csq,disc, d, t;

    tVec = sphere.center - ray.pixPt;
    v = tVec.dot(ray.direction);
    csq = tVec.dot(tVec);
    disc = pow(sphere.radius, 2) - (csq - pow(v, 2));

    if(disc > 0){
        d = sqrt(disc);
        t = v - d;

        if((t < ray.bestT) && (t > 0.00001)) {
            pt = ray.pixPt + t * ray.direction;

            ray.bestT = t;
            ray.bestSphere = sphere;
            ray.bestPt = ray.pixPt + t * ray.direction;
            ray.hitSphere = true;
            return true;
        }
    }

    ray.bestT = INF;
    return false;
}



Sphere rayFindSphere(Ray &ray, vector<Sphere> spheres){

    Sphere bestSphere = ray.bestSphere;
    for(auto s : spheres){
        if(raySphere(ray, s)){
            bestSphere = ray.bestSphere;
        }
    }

    return bestSphere;

}

Ray pixelRay(int i, int j, Camera camera){

    double x = 0;
    double y = 0;
    x = (i / (camera.resolution[0] - 1)) * (camera.bounds[2] - camera.bounds[0]) + camera.bounds[0];
    y = (j / (camera.resolution[1] - 1)) * (camera.bounds[3] - camera.bounds[1]) + camera.bounds[1];

    Eigen::Vector3d pixPt;
    Eigen::Vector3d rayVec;
    pixPt << camera.eyePt + (-camera.distance * camera.wVec) + (x * camera.uVec) + (y * camera.vVec);
    rayVec = (pixPt - camera.eyePt).normalized();

    Ray retRay;
    retRay.pixPt = pixPt;
    retRay.direction = rayVec.normalized();
    retRay.bestT = INF;
    retRay.hitSphere = false;
    retRay.hitTriangle = false;

    return retRay;
}

void placeCamera(Camera &camera){

    camera.upVec = camera.upVec.normalized();
    camera.wVec = (camera.eyePt - camera.lookAtVec).normalized();
    camera.uVec = (camera.upVec.cross(camera.wVec)).normalized();
    camera.vVec = (camera.wVec.cross(camera.uVec)).normalized();

}

Model rotate(Model &model){

    Model newModel;
    newModel.vertices = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d xform, xformt, rotation;
    Eigen::Matrix3d finalMatrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d axis;
    Eigen::Vector3d wm,mm,um,vm;

    //Make a 3d vector out of rotation axis
    axis << model.rotAxis(0) , model.rotAxis(1) , model.rotAxis(2);
    //Degrees to radians for angle
    double radianAngle = model.angle * (3.14159265359 / 180);
    //cout << "model angle: " << model.angle << '\n';
    //cout << "radian angle: " << radianAngle << '\n';

    //Normalize driver rotation axis
    wm = axis.normalized();

    mm = wm;


    //Find the minimum value (preferably z), change to 1
    bool changed = false;
    for(size_t i = 2; i > 0; i--) {
        if( (wm(i) == wm.minCoeff()) && (!changed) ){
            mm(i) = 1.0;
            changed = true;

        }
        else continue;
    }

    //Cross W x M into U and then normalize
    um = (wm.cross(mm)).normalized();

    vm = wm.cross(um).normalized();

    //Properly orient the matrix
    xform << um, vm, wm;
    xform.transposeInPlace();

    //Form transpose (inverse) matrix
    xformt = xform;
    xformt.transposeInPlace();

    //Form rotation matrix
    double topLeft, topRight, midLeft, midRight;
    topLeft = cos(radianAngle);
    topRight = -sin(radianAngle);
    midLeft = sin(radianAngle);
    midRight = cos(radianAngle);

    Eigen::Vector3d topRow, midRow, botRow;

    topRow << topLeft, topRight, 0;
    midRow << midLeft, midRight, 0;
    botRow << 0, 0, 1;
    rotation << topRow, midRow, botRow;
    rotation.transposeInPlace();

    finalMatrix = xformt * rotation * xform;

    for(int i = 0; i < finalMatrix.rows(); i++){
        for(size_t j = 0; j < 3; j++) {
            newModel.vertices(i, j) = finalMatrix(i, j);
        }
    }

    return newModel;



}

Model scale(Model &model){

    Model newModel;
    newModel.vertices = Eigen::Matrix4d::Identity();
    newModel.vertices(0,0) = newModel.vertices(0,0) * model.scale;
    newModel.vertices(1,1) = newModel.vertices(1,1) * model.scale;
    newModel.vertices(2,2) = newModel.vertices(2,2) * model.scale;

    return newModel;

}

Model translate(Model &model){

    Model newModel;
    newModel.vertices = Eigen::Matrix4d::Identity();
    newModel.vertices(0,3) = newModel.vertices(0,3) + model.translation(0);
    newModel.vertices(1,3) = newModel.vertices(1,3) + model.translation(1);
    newModel.vertices(2,3) = newModel.vertices(2,3) + model.translation(2);

    return newModel;

}

void transformObjs(vector<Model> &modelVec){

    for(size_t i = 0; i < modelVec.size(); i++) {

        Model tvex, svex, rvex;

        //form translation matrix
        tvex = translate(modelVec[i]);
        //form scale matrix
        svex = scale(modelVec[i]);
        //form rotation matrix
        rvex = rotate(modelVec[i]);

        Eigen::Matrix4d transformed;

        //form transformation matrix
        transformed = tvex.vertices * svex.vertices * rvex.vertices;
        //cout << transformed << '\n';

        Eigen::MatrixXd finalMatrix = modelVec[i].vertices;
        Eigen::Vector4d ogVert;
        Eigen::Vector4d xVert;

        //multiply vectors by transformation matrix
        for(int j = 0; j < finalMatrix.rows(); j++){
            ogVert = finalMatrix.row(j);
            xVert = transformed * ogVert;
            finalMatrix.row(j) = xVert;

        }
        //cout << finalMatrix << '\n';
        finalMatrix.conservativeResize(finalMatrix.rows(),finalMatrix.cols() - 1);
        modelVec[i].vertices = finalMatrix;
        //cout << finalMatrix << '\n';
    }

}

void parseMtls(vector<Model> &modelVec){
    for(size_t i = 0; i < modelVec.size(); i++){
        if(!modelVec[i].mtlFile.empty()){
            //open mtl file
            ifstream file;
            file.open(modelVec[i].mtlFile);
            string line;

            regex a("(^Ka)( .*)");
            regex d("(^Kd)( .*)");
            regex s("(^Ks)( .*)");
            regex p("(^Ns)( .*)");

            while(getline(file, line)){
                if(regex_match(line, a)){
                    vector<double> buffVec;
                    string buffer;
                    stringstream ss(line);

                    while(ss >> buffer){
                        buffVec.push_back(atof(buffer.c_str()));
                    }
                    modelVec[i].ka << buffVec[1], buffVec[2], buffVec[3];
//                    cout << modelVec[i].ka << '\n';
//                    cout << '\n';
                }
                if(regex_match(line, d)){
                    vector<double> buffVec;
                    string buffer;
                    stringstream ss(line);

                    while(ss >> buffer){
                        buffVec.push_back(atof(buffer.c_str()));
                    }
                    modelVec[i].kd << buffVec[1], buffVec[2], buffVec[3];
//                    cout << modelVec[i].kd << '\n';
//                    cout << '\n';
                }
                if(regex_match(line, s)){
                    vector<double> buffVec;
                    string buffer;
                    stringstream ss(line);

                    while(ss >> buffer){
                        buffVec.push_back(atof(buffer.c_str()));
                    }
                    modelVec[i].ks << buffVec[1], buffVec[2], buffVec[3];
//                    cout << modelVec[i].ks << '\n';
                }
                if(regex_match(line, p)){
                    vector<double> buffVec;
                    string buffer;
                    stringstream ss(line);

                    while(ss >> buffer){
                        buffVec.push_back(atof(buffer.c_str()));
                    }
                    modelVec[i].phong = buffVec[1];
//                    cout << modelVec[i].phong << '\n';
                }
            }
        }
    }
}

void parseObjs(vector<Model> &modelVec){

    for(size_t i = 0; i < modelVec.size(); i++){

        //Regexs for each line of obj
        regex coms ("(^#)( .*)");
        regex mtllib("(^mtllib)( .*)");
        regex verts ("(^v)( .*)");
        regex vertNorms ("(^vn)( .*)");
        regex faces ("(^f)( .*)");

        ifstream file;
        file.open(modelVec[i].filename);
        string line;

        //vectors to hold each type of line to parse
        vector<Eigen::Vector4d> parsedVerts;
        vector<vector<double> > parsedVns;
        vector<vector<int> >parsedFaceRefs;
        string parsedMtl;

        //populate those vectors
        while(getline(file, line)){

            //parse comments
            if(regex_match(line, coms)) {
                modelVec[i].comments.push_back(line);
            }

            //parse mtllib
            if(regex_match(line, mtllib)) {
                string buffer;
                stringstream ss(line);

                while(ss >> buffer){
                    modelVec[i].mtlFile = buffer;
                }
            }

            //parse vertices
            if(regex_match(line, verts)) {
                vector<double> buffVec;
                string buffer;
                stringstream ss(line);

                while(ss >> buffer){
                    buffVec.push_back(atof(buffer.c_str()));
                }

                Eigen::Vector4d vec;
                vec << buffVec[1], buffVec[2], buffVec[3], 1.0;

                parsedVerts.push_back(vec);

            }

            //parse vertex normals
            if(regex_match(line, vertNorms)) {
                vector<double> buffVec;
                string buffer;
                line = line.substr(2,line.size()-1);
                stringstream ss(line);

                while(ss >> buffer){
                    //handle negatives
                    if(buffer.substr(0,1) != "-") {
                        buffVec.push_back(atof(buffer.c_str()));
                    }
                    else{
                        buffVec.push_back(atof(buffer.c_str()));
                    }

                }
                parsedVns.push_back(buffVec);

            }

            //parse face references
            if(regex_match(line, faces)) {
                vector<int> buffVec;
                string buffer;
                line = line.substr(2,line.size()-1);
                stringstream ss(line);

                while(ss >> buffer){
                    //handle negatives
                    if(buffer.substr(0,1) != "-") {
                        buffVec.push_back(atoi(buffer.substr(0,1).c_str()));
                    }
                    else{
                        buffVec.push_back(atoi(buffer.substr(0,1).c_str()));
                    }

                }
                parsedFaceRefs.push_back(buffVec);

            }

        }

        file.close();

        Eigen::MatrixXd allVerts;
        for(size_t j = 0; j < parsedVerts.size(); j++){
            allVerts.conservativeResize(allVerts.rows() + 1, 4);
            Eigen::Vector4d v4d;

            //load vertices into Vector4ds
            v4d << parsedVerts[j][0], parsedVerts[j][1], parsedVerts[j][2], 1;
            //load Vector4ds into MatrixXd
            allVerts.row(j) = v4d;

        }
        modelVec[i].vertices = allVerts;

        //parse vertex norms
        Eigen::MatrixXd allVertNorms;
        for(size_t j = 0; j < parsedVns.size(); j++){
            allVertNorms.conservativeResize(allVertNorms.rows() + 1, 3);
            Eigen::Vector3d v3d;

            //load vertex normals into Vector3ds
            v3d << parsedVns[j][0], parsedVns[j][1], parsedVns[j][2];
            //load Vector3ds into MatrixXd
            allVertNorms.row(j) = v3d;

        }
        modelVec[i].vertexNorms = allVertNorms;

        //parse face references
        vector<Eigen::Vector3d> allFaceRefs;
        for(size_t j = 0; j < parsedFaceRefs.size(); j++){
            Eigen::Vector3d v3d;

            //load face refs into Vector3ds
            v3d << parsedFaceRefs[j][0], parsedFaceRefs[j][1], parsedFaceRefs[j][2];
            //load Vector3ds into vector of Vector3ds
            allFaceRefs.push_back(v3d);

        }
        modelVec[i].faceRefs = allFaceRefs;

        //parse faces (triangles) * .7
        vector<Triangle> allFaces;
        for(size_t j = 0; j < modelVec[i].faceRefs.size(); j++) {
            Triangle t;

            //add vertex A's xyz
            Eigen::Vector3d a;
            double ax = modelVec[i].vertices(modelVec[i].faceRefs[j](0)-1,0);
            double ay = modelVec[i].vertices(modelVec[i].faceRefs[j](0)-1,1);
            double az = modelVec[i].vertices(modelVec[i].faceRefs[j](0)-1,2);
            a << ax, ay, az;

            t.vertA = a;

            //add vertex B's xyz
            Eigen::Vector3d b;
            double bx = modelVec[i].vertices(modelVec[i].faceRefs[j](1)-1,0);
            double by = modelVec[i].vertices(modelVec[i].faceRefs[j](1)-1,1);
            double bz = modelVec[i].vertices(modelVec[i].faceRefs[j](1)-1,2);
            b << bx, by, bz;

            t.vertB = b;

            //add vertex C's xyz
            Eigen::Vector3d c;
            double cx = modelVec[i].vertices(modelVec[i].faceRefs[j](2)-1,0);
            double cy = modelVec[i].vertices(modelVec[i].faceRefs[j](2)-1,1);
            double cz = modelVec[i].vertices(modelVec[i].faceRefs[j](2)-1,2);
            c << cx, cy, cz;

            t.vertC = c;

            allFaces.push_back(t);

        }

        //populate faces in model
        modelVec[i].faces = allFaces;

    }
}

void parseDriver(Scene &scene, vector<string> &v){

    Camera c;
    vector<Light> lt;
    vector<Sphere> sph;
    vector<Model> mv;
    Eigen::Vector3d amb;


    for(size_t i = 0; i < v.size(); i++){

        //Allocate all camera elements

        if(v[i] == "eye"){
            c.eyePt << atof(v[i+1].c_str()), atof(v[i+2].c_str()), atof(v[i+3].c_str());
        }
        if(v[i] == "look"){
            c.lookAtVec << atof(v[i+1].c_str()), atof(v[i+2].c_str()), atof(v[i+3].c_str());
        }
        if(v[i] == "up"){
            c.upVec << atof(v[i+1].c_str()), atof(v[i+2].c_str()), atof(v[i+3].c_str());
        }
        if(v[i] == "d"){
            c.distance = atof(v[i+1].c_str());
        }
        if(v[i] == "bounds"){
            c.bounds.push_back(atof(v[i+1].c_str()));
            c.bounds.push_back(atof(v[i+2].c_str()));
            c.bounds.push_back(atof(v[i+3].c_str()));
            c.bounds.push_back(atof(v[i+4].c_str()));
        }
        if(v[i] == "res"){
            c.resolution.push_back(atof(v[i+1].c_str()));
            c.resolution.push_back(atof(v[i+2].c_str()));
        }
        if(v[i] == "recursionLevel"){
            scene.recursionLevel = atof(v[i+1].c_str());
        }
        if(v[i] == "ambient"){
            amb << atof(v[i+1].c_str()),atof(v[i+2].c_str()),atof(v[i+3].c_str());
        }
        if(v[i] == "light"){
            Light light;
            light.position << atof(v[i+1].c_str()), atof(v[i+2].c_str()), atof(v[i+3].c_str());
            light.w = atof(v[i+4].c_str());
            light.emission << atof(v[i+5].c_str()), atof(v[i+6].c_str()), atof(v[i+7].c_str());

            lt.push_back(light);
        }
        if(v[i] == "sphere"){
            Sphere sphere;
            sphere.center << atof(v[i+1].c_str()), atof(v[i+2].c_str()), atof(v[i+3].c_str());
            sphere.radius = atof(v[i+4].c_str());
            sphere.ambient << atof(v[i+5].c_str()), atof(v[i+6].c_str()), atof(v[i+7].c_str());
            sphere.diffuse << atof(v[i+8].c_str()), atof(v[i+9].c_str()), atof(v[i+10].c_str());
            sphere.specular << atof(v[i+11].c_str()), atof(v[i+12].c_str()), atof(v[i+13].c_str());
            sphere.attenuation << atof(v[i+14].c_str()), atof(v[i+15].c_str()), atof(v[i+16].c_str());
            sphere.opacity << atof(v[i+17].c_str()), atof(v[i+18].c_str()), atof(v[i+19].c_str());
            sphere.phong = atof(v[i+20].c_str());
            sphere.eta = atof(v[i+21].c_str());
            sphere.isReal = true;

            sph.push_back(sphere);
        }
        if(v[i] == "model"){
            Model model;
            model.rotAxis << atof(v[i+1].c_str()), atof(v[i+2].c_str()), atof(v[i+3].c_str()), 1;
            model.angle = atof(v[i+4].c_str());
            model.scale = atof(v[i+5].c_str());
            model.translation << atof(v[i+6].c_str()), atof(v[i+7].c_str()), atof(v[i+8].c_str()), 1;
            model.filename = v[i+9];

            mv.push_back(model);
        }
        if(v[i] == "eta_outside"){
            scene.eta_outside = atof(v[i+1].c_str());
        }

    }


    for(size_t i = 0; i < c.resolution[0]; i++){
        vector<Eigen::Vector3d> row;

        //Fill image with 0s
        for(size_t j = 0; j < c.resolution[1]; j++){
            Eigen::Vector3d nil;
            nil << 0,0,0;
            row.push_back(nil);
        }

        scene.image.push_back(row);

    }

    //add all parsed elements to Scene object
    scene.camera = c;
    scene.ambiance = amb;
    scene.lights = lt;
    scene.models = mv;
    scene.spheres = sph;

}