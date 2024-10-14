#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "../external/json.hpp"
#include "../external/simpleppm.h"

class Geometry;
class Light;
class Output;

class RayTracer {
    private:
    public:
        RayTracer(nlohmann::json& j);
        void run();
        std::vector<Geometry>* geometryList;
        std::vector<Light>* lightList;
        std::vector<Output>* outputList;
};

class Ray {
    private:
    public:
        Eigen::Vector3f origin;
        Eigen::Vector3f direction;
        Ray();
        Ray(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction);

        Geometry *closestGeometry = nullptr;
        int closestGeometryIndex = -1;
        int previousGeometryIndex = -1;
        float t = INFINITY;
        float distToLight = -66; 
};

class Geometry {
    public:
        Geometry();
        std::string type;
        float ka;
        float kd;
        float ks;
        float pc;
        Eigen::Vector3f ac;
        Eigen::Vector3f dc;
        Eigen::Vector3f sc;
        //Optional
        float transform[16];
        //Sphere
        float radius;
        Eigen::Vector3f centre;
        //Rectangle
        Eigen::Vector3f p1;
        Eigen::Vector3f p2;
        Eigen::Vector3f p3;
        Eigen::Vector3f p4;

        Eigen::Vector3f rectNormal;
        bool hasNormal = false;

        Eigen::Vector3f getMinBounds();
        Eigen::Vector3f getMaxBounds();
};

class Light {
    public:
        Light();
        std::string type;
        Eigen::Vector3f id;
        Eigen::Vector3f is;
        //Optional
        float transform[16];
        //Light
        Eigen::Vector3f centre;
        //Area
        Eigen::Vector3f p1;
        Eigen::Vector3f p2;
        Eigen::Vector3f p3;
        Eigen::Vector3f p4;
};

class Output {
    public:
        Output();
        std::string filename;
        unsigned int size[2];
        float fov;
        Eigen::Vector3f up;
        Eigen::Vector3f lookat;
        Eigen::Vector3f ai;
        Eigen::Vector3f bkc;
        Eigen::Vector3f centre;
        //Optional
        Eigen::Vector2i raysperpixel;
        bool antialiasing = false;
        bool twosiderender = false;
        bool globalillum = false;

        unsigned int speedup = 1;
        int maxbounces = 0;
        float probterminate = 0.33;
};

class BoundingBox {
    public:
        BoundingBox();
        BoundingBox(std::vector<Geometry>& geometryList);
        Eigen::Vector3f minimumBox;
        Eigen::Vector3f maximumBox;
};
