#include "RayTracer.h"
#include "jsonParser.h"

using namespace std;

float clamp(float v) {
    return min(1.0f, max(v, 0.0f));
}

int savePPM(Output& output, vector<Geometry>* geometryList, vector<Light>* lightList, BoundingBox& boundingBox);

RayTracer::RayTracer(nlohmann::json& j) {
    geometryList = new vector<Geometry>;
    lightList = new vector<Light>;
    outputList = new vector<Output>;

    parseGeometry(j, geometryList);
    parseLight(j, lightList);
    parseOutput(j, outputList);

}

BoundingBox::BoundingBox(vector<Geometry>& geometryList) {
    Eigen::Vector3f boxMin(0, 0, 0);
    Eigen::Vector3f boxMax(0, 0, 0);
    float currentMinX;
    float currentMinY;
    float currentMinZ;

    float currentMaxX;
    float currentMaxY;
    float currentMaxZ;

    int i = 0;
    for (Geometry geometry : geometryList) {
        Eigen::Vector3f minimum = geometry.getMinBounds();
        Eigen::Vector3f maximum = geometry.getMaxBounds();

        currentMinX = minimum[0] < maximum[0] ? minimum[0] : maximum[0];
        currentMinY = minimum[1] < maximum[1] ? minimum[1] : maximum[1];
        currentMinZ = minimum[2] < maximum[2] ? minimum[2] : maximum[2];

        currentMaxX = minimum[0] > maximum[0] ? minimum[0] : maximum[0];
        currentMaxY = minimum[1] > maximum[1] ? minimum[1] : maximum[1];
        currentMaxZ = minimum[2] > maximum[2] ? minimum[2] : maximum[2];

        if (i == 0) {
            boxMin[0] = currentMinX;
            boxMin[1] = currentMinY;
            boxMin[2] = currentMinZ;
            boxMax[0] = currentMaxX;
            boxMax[1] = currentMaxY; 
            boxMax[2] = currentMaxZ;
        } else {
            if (boxMin[0] > currentMinX) 
                boxMin[0] = currentMinX;
            if (boxMin[1] > currentMinY) 
                boxMin[1] = currentMinY;
            if (boxMin[2] > currentMinZ) 
                boxMin[2] = currentMinZ;

            if (boxMax[0] < currentMaxX) 
                boxMax[0] = currentMaxX;
            if (boxMax[1] < currentMaxY) 
                boxMax[1] = currentMaxY;
            if (boxMax[2] < currentMaxZ) 
                boxMax[2] = currentMaxZ;
        }
        i++;
    }
    minimumBox = boxMin;
    maximumBox = boxMax;

}

void RayTracer::run() {
    BoundingBox boundingBox(*geometryList);

    for (Output output : *outputList) {
        savePPM(output, geometryList, lightList, boundingBox);
    }
}

Eigen::Vector3f Geometry::getMinBounds() {
    Eigen::Vector3f boundingBoxMin;
    if (this->type == "sphere") {
        boundingBoxMin = this->centre - Eigen::Vector3f(this->radius, this->radius, this->radius);
    } else if (this->type == "rectangle") {
        boundingBoxMin = this->p1;
    }
    return boundingBoxMin;
}
Eigen::Vector3f Geometry::getMaxBounds() {
    Eigen::Vector3f boundingBoxMax;
    if (this->type == "sphere") {
        boundingBoxMax = this->centre + Eigen::Vector3f(this->radius, this->radius, this->radius);
    } else if (this->type == "rectangle") {
        boundingBoxMax = this->p3;
    }
    return boundingBoxMax;
}

float intersectGeometry(const Ray& ray, Geometry& geometry) {
    if (geometry.type == "sphere") {
        Eigen::Vector3f oc = ray.origin - geometry.centre;
            
        auto a = ray.direction.dot(ray.direction);
        auto b = 2.0 * oc.dot(ray.direction);
        auto c = oc.dot(oc) - geometry.radius*geometry.radius;
        auto discriminant = b*b - 4*a*c;

        auto tPlus = (-b + sqrt(discriminant)) / 2*a;
        auto tMinus = (-b - sqrt(discriminant)) / 2*a;

        auto tFound = discriminant > 0 ? true : false;
        if (tFound) {
            if (tPlus < 0 && tMinus < 0) {
                return -999;
            } else if (tPlus < 0 || tMinus < 0) {
                auto tInFront = tPlus < 0 ? tMinus : tPlus;
                return tInFront;
            } else {
                auto tInFront = tPlus < tMinus ? tPlus : tMinus;
                return tInFront;
            }
        }
    } else if (geometry.type == "rectangle") {
        Eigen::Vector3f normalOfPlane;
        if (geometry.hasNormal != true) {
            auto p1p2 = geometry.p2 - geometry.p1;
            auto p1p4 = geometry.p4 - geometry.p1;
            normalOfPlane = p1p2.cross(p1p4);
            geometry.rectNormal = normalOfPlane;
            geometry.hasNormal = true;
        } else {
            normalOfPlane = geometry.rectNormal;
        }
        
        float d = -(normalOfPlane[0] * geometry.p1[0] + normalOfPlane[1] * geometry.p1[1] + normalOfPlane[2] * geometry.p1[2]); 
        
        auto isParallel = normalOfPlane.dot(ray.direction) == 0 ? true : false;

        if (isParallel) {
            return -999;
        } else {
            auto t = - ((normalOfPlane.dot(ray.origin) + d) / normalOfPlane.dot(ray.direction));
            auto behindRay = t < 0;
            if (behindRay) {
                return -999;
            } else {
                auto intersectionPointPlane = ray.origin + t * ray.direction;
                auto signP1P2 = normalOfPlane.dot((geometry.p2 - geometry.p1).cross(intersectionPointPlane - geometry.p1));
                auto signP2P3 = normalOfPlane.dot((geometry.p3 - geometry.p2).cross(intersectionPointPlane - geometry.p2));
                auto signP3P4 = normalOfPlane.dot((geometry.p4 - geometry.p3).cross(intersectionPointPlane - geometry.p3));
                auto signP4P1 = normalOfPlane.dot((geometry.p1 - geometry.p4).cross(intersectionPointPlane - geometry.p4));
                
                if (signP1P2 > 0 && signP2P3 > 0 && signP3P4 > 0 && signP4P1 > 0) {
                    return t;
                }
                
            }
        }
    }

    return -999;
}

bool intersect(Ray& ray, vector<Geometry>* geometryList) {

    for (int i = 0; i < geometryList->size(); i++) {
        float currentT = intersectGeometry(ray, geometryList->at(i));
        if (ray.previousGeometryIndex == i) {
            ray.previousGeometryIndex = -1;
            continue;
        }
        if (currentT != -999 && currentT < ray.t) {
            if (ray.distToLight != -66) {
                // ShadowRay
                auto hitPoint = ray.origin + currentT * ray.direction;
                auto distanceToHit = (hitPoint - ray.origin).norm();
                if (distanceToHit > ray.distToLight) {
                    continue;
                } else {
                    return true;
                }
            }
            ray.t = currentT;
            ray.closestGeometryIndex = i;
            ray.closestGeometry = &(geometryList->at(i));
        }
    } 
    
    return (ray.closestGeometry != nullptr); 
} 

bool intersectBoundingBox(Ray& ray, BoundingBox& boundingBox) {

    float fracX = 1.0 / ray.direction[0];
    float fracY = 1.0 / ray.direction[1];
    float fracZ = 1.0 / ray.direction[2];
    
    float t1 = (boundingBox.minimumBox[0] - ray.origin[0]) * fracX;
    float t2 = (boundingBox.maximumBox[0] - ray.origin[0]) * fracX;
    float t3 = (boundingBox.minimumBox[1] - ray.origin[1]) * fracY;
    float t4 = (boundingBox.maximumBox[1] - ray.origin[1]) * fracY;
    float t5 = (boundingBox.minimumBox[2] - ray.origin[2]) * fracZ;
    float t6 = (boundingBox.maximumBox[2] - ray.origin[2]) * fracZ;

    float tMin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    float tMax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

    return !((tMax < 0) || (tMin > tMax));

}

Eigen::Vector3f castRay(Ray& ray, Output& output, vector<Geometry>* geometryList, vector<Light>* lightList) {
    Eigen::Vector3f finalColor;

    if (intersect(ray, geometryList)) {

        Eigen::Vector3f pointHit = ray.origin + ray.t * ray.direction;
        Eigen::Vector3f normalHit;
        if (ray.closestGeometry->type == "sphere") {
            normalHit = (pointHit - ray.closestGeometry->centre).normalized();
        } else {
            normalHit = (ray.closestGeometry->rectNormal).normalized(); 
        }

        Eigen::Vector3f ambient(0, 0, 0);
        Eigen::Vector3f diffuse(0, 0, 0);
        Eigen::Vector3f specular(0, 0, 0);
        Eigen::Vector3f diffuseArea(0, 0, 0);
        Eigen::Vector3f specularArea(0, 0, 0);
        float diffuseAcc = 0;
        float specularAcc = 0;
        float diffuseAccArea = 0;
        float specularAccArea = 0;
        ambient = ray.closestGeometry->ac * ray.closestGeometry->ka;

        for (Light light : *lightList) {
            if (light.type == "area") {
                auto corner = light.p1;
                auto v1 = (light.p4 - light.p1);
                auto v1Norm = v1.norm();;
                
                auto v2 = (light.p2 - light.p1);
                auto v2Norm = v2.norm();
                float total = 0;

                int inc = 1;
                int inc2 = 1;
                if (output.antialiasing == true) {
                    float dice = rand() / (float)RAND_MAX;
                    float dice2 = rand() / (float)RAND_MAX;
                    inc = v1Norm * dice;
                    inc2 = v2Norm * dice2;
                    if (inc == 0)
                        inc = 1;
                    if (inc2 == 0)
                        inc2 = 1;
                } else {
                    if (v1Norm > 15) {
                        inc = v1Norm * 0.11;
                    }
                    if (v2Norm > 15) {
                        inc2 = v2Norm * 0.11;
                    }
                }
                
            auto hit = 0;
                for (int i = 0; i <= v1Norm; i+= inc) {
                    for (int k = 0; k <= v2Norm; k+= inc2) {
                        auto lightPos = corner + (v1 / v1Norm) * i + (v2 / v2Norm) * k;
                        
                        Eigen::Vector3f lightDirection = (pointHit - lightPos).normalized();
                        
                        // SHADOWS
                        Ray shadowRay(pointHit, lightPos - pointHit);
                        shadowRay.distToLight = (lightPos - pointHit).norm();

                        vector<Geometry> shadowGeometryList;
                        shadowGeometryList = *geometryList;
                        if (ray.closestGeometryIndex != -1)
                            shadowGeometryList.erase(shadowGeometryList.begin() + ray.closestGeometryIndex);
                        if (intersect(shadowRay, &shadowGeometryList)) {
                            continue;
                        }
                        
                        diffuseAccArea += clamp(ray.closestGeometry->kd * -lightDirection.dot(normalHit));
                        diffuseArea = ray.closestGeometry->dc * diffuseAccArea;
                        
                        diffuseArea[0] *= light.id[0];
                        diffuseArea[1] *= light.id[1];
                        diffuseArea[2] *= light.id[2];
                        
                        auto h = (-lightDirection + (-ray.direction)) / (-lightDirection + (-ray.direction)).norm();
                        auto beta = h.dot(normalHit);
                        
                        specularAccArea += clamp(ray.closestGeometry->ks * pow(beta, ray.closestGeometry->pc));
                        specularArea = ray.closestGeometry->sc * specularAccArea;
                        
                        specularArea[0] *= light.is[0];
                        specularArea[1] *= light.is[1];
                        specularArea[2] *= light.is[2];
                        
                    }
                }

                diffuseArea = diffuseArea / ((v1Norm+1) * (v2Norm+1));
                specularArea = specularArea / ((v1Norm+1) * (v2Norm+1));
                
            } else if (light.type == "point") {

                Eigen::Vector3f lightDirection = (pointHit - light.centre).normalized();

                // SHADOWS
                Eigen::Vector3f lightDir = (light.centre - pointHit);
                Ray shadowRay(pointHit, lightDir);
                shadowRay.distToLight = lightDir.norm();
                vector<Geometry> shadowGeometryList;
                shadowGeometryList = *geometryList;
                shadowGeometryList.erase(shadowGeometryList.begin() + ray.closestGeometryIndex);
                if (intersect(shadowRay, &shadowGeometryList)) {
                    continue;
                }
                
                diffuseAcc += clamp(ray.closestGeometry->kd * -lightDirection.dot(normalHit));
                diffuse = ray.closestGeometry->dc * diffuseAcc;
                
                diffuse[0] *= light.id[0];
                diffuse[1] *= light.id[1];
                diffuse[2] *= light.id[2];

                auto h = (-lightDirection + (-ray.direction)) / (-lightDirection + (-ray.direction)).norm();
                auto beta = h.dot(normalHit);
                specularAcc += clamp(ray.closestGeometry->ks * pow(beta, ray.closestGeometry->pc));
                specular = ray.closestGeometry->sc * specularAcc;
                
                specular[0] *= light.is[0];
                specular[1] *= light.is[1];
                specular[2] *= light.is[2];
                
            }
        }
        
        auto df = (diffuse + diffuseArea);
        auto sf = (specular + specularArea);
        
        finalColor = ambient + df + sf;

    } else {
        finalColor[0] = output.bkc[0]; // R
        finalColor[1] = output.bkc[1]; // G
        finalColor[2] = output.bkc[2]; // B
    }

    return finalColor;
}

Eigen::Vector3f castRayGlobal(Ray& ray, Output& output, vector<Geometry>* geometryList, vector<Light>* lightList, int bounce) {
    Eigen::Vector3f finalColor;

    if (intersect(ray, geometryList)) {

        Eigen::Vector3f pointHit = ray.origin + ray.t * ray.direction;
        Eigen::Vector3f normalHit;
        if (ray.closestGeometry->type == "sphere") {
            normalHit = (pointHit - ray.closestGeometry->centre).normalized(); 
        } else {
            normalHit = (ray.closestGeometry->rectNormal).normalized(); 
        }
        
        Eigen::Vector3f diffuse(0, 0, 0);
        Eigen::Vector3f diffuseArea(0, 0, 0);
        float diffuseAcc = 0;
        float diffuseAccArea = 0;

        float dice = rand() / (float)RAND_MAX;
        bool russianRoulette = (dice <= output.probterminate);
        if (russianRoulette || (bounce == output.maxbounces)) {

            for (Light light : *lightList) {
                if (light.type == "area") {
                    auto corner = light.p1;
                    auto v1 = (light.p4 - light.p1);
                    auto v1Norm = v1.norm();;
                    
                    auto v2 = (light.p2 - light.p1);
                    auto v2Norm = v2.norm();
                    float total = 0;

                    int inc = 1;
                    int inc2 = 1;
                    if (v1Norm > 1) {
                        inc = v1Norm * 0.51;
                    }
                    if (v2Norm > 1) {
                        inc2 = v2Norm * 0.51;
                    }

                    for (int i = 0; i <= v1Norm; i+= inc) {
                        for (int k = 0; k <= v2Norm; k+= inc2) {
                            auto lightPos = corner + (v1 / v1Norm) * i + (v2 / v2Norm) * k;

                            Eigen::Vector3f lightDirection = (pointHit - lightPos).normalized();
                            
                            // SHADOWS
                            Ray shadowRay(pointHit, lightPos - pointHit);
                            shadowRay.distToLight = (lightPos - pointHit).norm();
                            vector<Geometry> shadowGeometryList;
                            shadowGeometryList = *geometryList;
                            if (ray.closestGeometryIndex != -1)
                                shadowGeometryList.erase(shadowGeometryList.begin() + ray.closestGeometryIndex);
                            if (intersect(shadowRay, &shadowGeometryList)) {
                                continue;
                            }
                            
                            diffuseAccArea += clamp(ray.closestGeometry->kd * -lightDirection.dot(normalHit));
                            diffuseArea = ray.closestGeometry->dc * diffuseAccArea;
                            
                            diffuseArea[0] *= light.id[0];
                            diffuseArea[1] *= light.id[1];
                            diffuseArea[2] *= light.id[2];

                        }
                    }

                    diffuseArea = diffuseArea / ((v1Norm+1) * (v2Norm+1));
                    
                } else if (light.type == "point") {

                    Eigen::Vector3f lightDirection = (pointHit - light.centre).normalized();

                    // SHADOWS
                    Eigen::Vector3f lightDir = (light.centre - pointHit);

                    Ray shadowRay(pointHit, lightDir);
                    shadowRay.distToLight = lightDir.norm();

                    vector<Geometry> shadowGeometryList;
                    shadowGeometryList = *geometryList;
                    shadowGeometryList.erase(shadowGeometryList.begin() + ray.closestGeometryIndex);
                    if (intersect(shadowRay, &shadowGeometryList)) {
                        finalColor[0] = 0; // R
                        finalColor[1] = 0; // G
                        finalColor[2] = 0; // B
                        return finalColor;
                    }
                    
                    diffuseAcc += clamp(-lightDirection.dot(normalHit));
                    diffuse = ray.closestGeometry->dc * diffuseAcc;
                    diffuse[0] *= light.id[0];
                    diffuse[1] *= light.id[1];
                    diffuse[2] *= light.id[2];
                }
            }
            auto df = (diffuse + diffuseArea);
            finalColor = df;
            return finalColor;
        }

        float num1 = ((double)rand() / (double)RAND_MAX);
        float num2 = ((double)rand() / (double)RAND_MAX);

        float r = sqrt(num1);
        float theta = 2 * PI * num2;
    
        float x = r * cos(theta);
        float y = r * sin(theta);
        Eigen::Vector3f sampleDir(x, y, sqrt(1 - num1));
        Eigen::Vector3f up(0, 0, 1);
        auto angle = acos(up.dot(normalHit));
        auto axis = up.cross(normalHit);

        if (up == normalHit) {
            //
        } else if (up == -normalHit) {
            //
            sampleDir = -1 * sampleDir;
        } else {
            Eigen::AngleAxisf aa(angle, axis);
            Eigen::Quaternionf q(aa);
            sampleDir = q * sampleDir;
        }

        Ray newRay(pointHit, sampleDir);
        newRay.previousGeometryIndex = ray.closestGeometryIndex;

        float cos_theta = newRay.direction.dot(normalHit);
        float BRDF = 1 / PI;

        float pdfCosine = cos_theta / PI;
        Eigen::Vector3f incoming = castRayGlobal(newRay, output, geometryList, lightList, bounce + 1);

        if (bounce == 0) {
            auto finalColorG = ((BRDF * incoming * cos_theta) / pdfCosine) * (1 / (1 - output.probterminate)); 
            pow(finalColorG[0], 0.4545);
            pow(finalColorG[1], 0.4545);
            pow(finalColorG[2], 0.4545);
            return finalColorG;
        } else {
            return ((BRDF * incoming * cos_theta) / pdfCosine) * (1 / (1 - output.probterminate));
        }
        
    } else {
        finalColor[0] = 0; // R
        finalColor[1] = 0; // G
        finalColor[2] = 0; // B
        return finalColor;
    }

}

int savePPM(Output& output, vector<Geometry>* geometryList, vector<Light>* lightList, BoundingBox& boundingBox) {
    int dimx = output.size[0];
    int dimy = output.size[1];
    
    auto sceneMiddle = output.centre + output.lookat;
    auto sceneUpperMiddle = sceneMiddle + tan(output.fov / 2) * output.up;
    auto rightVector = output.lookat.cross(output.up);
    auto deltaPixelSize = (2 * tan(output.fov / 2)) / dimy;
    auto upper_left_corner = sceneUpperMiddle - (dimx/2) * deltaPixelSize * rightVector;

    float deltaPixelSubSize;

    if (output.antialiasing == true || output.globalillum == true) {
        if (output.raysperpixel[1] > 0) { 
            deltaPixelSubSize = deltaPixelSize / output.raysperpixel[0];
        }
    }

    std::vector<double> buffer(3*dimx*dimy);

    srand(time(NULL));
    for (int j = 0; j < dimy; ++j) {
        for (int i = 0; i < dimx; ++i) {

            Eigen::Vector3f finalColor(0, 0, 0);

            if (output.antialiasing == true || output.globalillum == true) {
                if (output.raysperpixel[1] == 0) {
                    
                    for (int sample = 0; sample < output.raysperpixel[0]; sample++) {

                        auto r1 = ((double)rand() / (double)RAND_MAX);
                        auto r2 = ((double)rand() / (double)RAND_MAX);
                        auto x = deltaPixelSize * r1;
                        auto y = deltaPixelSize * r2;
                        auto randomAreaPixel = upper_left_corner +  (i * deltaPixelSize + x) * rightVector - (j * deltaPixelSize + y) * output.up;

                        auto rayDirectionOP = randomAreaPixel - output.centre;
                        Ray ray(output.centre, rayDirectionOP);
                        
                        if (output.speedup == 1) {
                            if (intersectBoundingBox(ray, boundingBox)) {
                                if (output.globalillum == true) {
                                    finalColor += castRayGlobal(ray, output, geometryList, lightList, 0);
                                } else {
                                    finalColor += castRay(ray, output, geometryList, lightList);
                                }
                            } else {
                                finalColor[0] += output.bkc[0]; // R
                                finalColor[1] += output.bkc[1]; // G
                                finalColor[2] += output.bkc[2]; // B
                            }
                        } else {
                            if (output.globalillum == true) {
                                finalColor += castRayGlobal(ray, output, geometryList, lightList, 0);
                            } else {
                                finalColor += castRay(ray, output, geometryList, lightList);
                            }
                        }
                    }

                    float div = 1 / float(output.raysperpixel[0]);
                    finalColor = finalColor * div;

                } else {

                    for (int x0 = 0; x0 < output.raysperpixel[0]; x0++) {
                        for (int y0 = 0; y0 < output.raysperpixel[0]; y0++) {
                            for (int sample = 0; sample < output.raysperpixel[1]; sample++) {
                                auto r1 = ((double)rand() / (double)RAND_MAX);
                                auto r2 = ((double)rand() / (double)RAND_MAX);
                                auto x = deltaPixelSubSize * r1;
                                auto y = deltaPixelSubSize * r2;
                                auto randomAreaPixel = upper_left_corner +  ((i * deltaPixelSize) + (x0 * deltaPixelSubSize) + x) * rightVector - ((j * deltaPixelSize) + (y0 * deltaPixelSubSize) + y) * output.up;
                                
                                auto rayDirectionOP = randomAreaPixel - output.centre;
                                Ray ray(output.centre, rayDirectionOP);
                                if (output.speedup == 1) {
                                    if (intersectBoundingBox(ray, boundingBox)) {
                                        if (output.globalillum == true) {
                                            finalColor += castRayGlobal(ray, output, geometryList, lightList, 0);
                                        } else {
                                            finalColor += castRay(ray, output, geometryList, lightList);
                                        }
                                    } else {
                                        finalColor[0] += output.bkc[0]; // R
                                        finalColor[1] += output.bkc[1]; // G
                                        finalColor[2] += output.bkc[2]; // B
                                    }
                                } else {
                                    if (output.globalillum == true) {
                                        finalColor += castRayGlobal(ray, output, geometryList, lightList, 0);
                                    } else {
                                        finalColor += castRay(ray, output, geometryList, lightList);
                                    }
                                }
                            }
                        }
                    }

                    float div = 1 / float(((output.raysperpixel[0] * output.raysperpixel[0] * output.raysperpixel[1])));
                    finalColor = finalColor * div;

                }
            } else {
                auto middleOfPixel = upper_left_corner +  (i * deltaPixelSize + deltaPixelSize/2) * rightVector - (j * deltaPixelSize + deltaPixelSize/2) * output.up;

                auto rayDirectionOP = middleOfPixel - output.centre;
                Ray ray(output.centre, rayDirectionOP);
                
                if (output.speedup == 1) {
                    if (intersectBoundingBox(ray, boundingBox)) {
                        finalColor = castRay(ray, output, geometryList, lightList);
                    } else {
                        finalColor[0] = output.bkc[0]; // R
                        finalColor[1] = output.bkc[1]; // G
                        finalColor[2] = output.bkc[2]; // B
                    }
                } else {
                    finalColor = castRay(ray, output, geometryList, lightList);
                }
            }

            buffer[3*j*dimx + 3*i + 0] = clamp(finalColor[0]); // R
            buffer[3*j*dimx + 3*i + 1] = clamp(finalColor[1]); // G
            buffer[3*j*dimx + 3*i + 2] = clamp(finalColor[2]); // B      
            
        }
    }

    save_ppm(output.filename, buffer, dimx, dimy);
    return 0;
}

Ray::Ray() { }
Ray::Ray(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction) { 
    this->origin = origin;   
    this->direction = direction.normalized(); 
}

BoundingBox::BoundingBox() { }
Geometry::Geometry() { }
Light::Light() { }
Output::Output() { }
