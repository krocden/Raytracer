#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace nlohmann;

const double PI = 3.14159265359;

bool parseGeometry(nlohmann::json &j, vector<Geometry>* Geometry);
bool parseLight(nlohmann::json &j);
bool parseOutput(nlohmann::json &j);

bool parseGeometry(json& j, vector<Geometry>* geometryList){
    int gc = 0;
    
    // use iterators to read-in array types
    for (auto itr = j["geometry"].begin(); itr!= j["geometry"].end(); itr++){
        std::string type;
        if(itr->contains("type")){
           // type = static_cast<std::string>((*itr)["type"]);
            type = (*itr)["type"].get<std::string>();
        } else {
            return false;
        }
        
        Geometry currentGeometry;

        if(itr->contains("pc")) {
            currentGeometry.pc = (*itr)["pc"].get<float>();
        }
        if(itr->contains("ka")) {
            currentGeometry.ka = (*itr)["ka"].get<float>();
        }
        if(itr->contains("kd")) {
            currentGeometry.kd = (*itr)["kd"].get<float>();
        }
        if(itr->contains("ks")) {
            currentGeometry.ks = (*itr)["ks"].get<float>();
        }

        Eigen::Vector3f ac(0,0,0);
        Eigen::Vector3f dc(0,0,0);
        Eigen::Vector3f sc(0,0,0);
        float transform[16];
        int i = 0;
        for (auto itrM =(*itr)["ac"].begin(); itrM!= (*itr)["ac"].end(); itrM++){
            if(i<3){
                ac[i++] = (*itrM).get<float>();
            }
        }
        i = 0;
        for (auto itrM =(*itr)["dc"].begin(); itrM!= (*itr)["dc"].end(); itrM++){
            if(i<3){
                dc[i++] = (*itrM).get<float>();
            }
        }
        i = 0;
        for (auto itrM =(*itr)["sc"].begin(); itrM!= (*itr)["sc"].end(); itrM++){
            if(i<3){
                sc[i++] = (*itrM).get<float>();
            }
        }
        
        if(itr->contains("transform")){
            i = 0;
            for (auto itrM =(*itr)["transform"].begin(); itrM!= (*itr)["transform"].end(); itrM++){
                if(i<16){
                    currentGeometry.transform[i++] = (*itrM).get<float>();
                }
            }
        }
        currentGeometry.ac = ac;
        currentGeometry.dc = dc;
        currentGeometry.sc = sc;

        if(type=="sphere"){
            currentGeometry.type = type;
            if(itr->contains("radius")) {
                currentGeometry.radius = (*itr)["radius"].get<float>();
            }

            Eigen::Vector3f centre(0,0,0);
            int i = 0;
            for (auto itr2 =(*itr)["centre"].begin(); itr2!= (*itr)["centre"].end(); itr2++){
                if(i<3){
                    centre[i++] = (*itr2).get<float>();
                }
            }
            currentGeometry.centre = centre;

        } else if(type=="rectangle"){
            currentGeometry.type = type;
            Eigen::Vector3f p1(0,0,0);
            Eigen::Vector3f p2(0,0,0);
            Eigen::Vector3f p3(0,0,0);
            Eigen::Vector3f p4(0,0,0);
            int i = 0;
            for (auto itr2 =(*itr)["p1"].begin(); itr2!= (*itr)["p1"].end(); itr2++){
                if(i<3){
                    p1[i++] = (*itr2).get<float>();
                }
            }
            i = 0;
            for (auto itr2 =(*itr)["p2"].begin(); itr2!= (*itr)["p2"].end(); itr2++){
                if(i<3){
                    p2[i++] = (*itr2).get<float>();
                }
            }
            i = 0;
            for (auto itr2 =(*itr)["p3"].begin(); itr2!= (*itr)["p3"].end(); itr2++){
                if(i<3){
                    p3[i++] = (*itr2).get<float>();
                }
            }
            i = 0;
            for (auto itr2 =(*itr)["p4"].begin(); itr2!= (*itr)["p4"].end(); itr2++){
                if(i<3){
                    p4[i++] = (*itr2).get<float>();
                }
            }
            currentGeometry.p1 = p1;
            currentGeometry.p2 = p2;
            currentGeometry.p3 = p3;
            currentGeometry.p4 = p4;
        }
        
        geometryList->push_back(currentGeometry);
        ++gc;
    }
    
    return true;
}

bool parseLight(json& j, vector<Light>* lightList){
    int lc = 0;
    
    // use iterators to read-in array types
    for (auto itr = j["light"].begin(); itr!= j["light"].end(); itr++){
        
        std::string type;
        if(itr->contains("type")){
            type = (*itr)["type"].get<std::string>();
        } else {
            return false;
        }
        
        Light currentLight;
        
        currentLight.type = type;

        Eigen::Vector3f id(0,0,0);
        Eigen::Vector3f is(0,0,0);
        int i = 0;
        for (auto itrM =(*itr)["id"].begin(); itrM!= (*itr)["id"].end(); itrM++){
            if(i<3){
                id[i++] = (*itrM).get<float>();
            }
        }
        i = 0;
        for (auto itrM =(*itr)["is"].begin(); itrM!= (*itr)["is"].end(); itrM++){
            if(i<3){
                is[i++] = (*itrM).get<float>();
            }
        }
        currentLight.id = id;
        currentLight.is = is;

        if(itr->contains("transform")){
            i = 0;
            for (auto itrM =(*itr)["transform"].begin(); itrM!= (*itr)["transform"].end(); itrM++){
                if(i<16){
                    currentLight.transform[i++] = (*itrM).get<float>();
                }
            }
        }

        if(type=="point"){
            Eigen::Vector3f centre(0,0,0);
            int i = 0;
            for (auto itr2 =(*itr)["centre"].begin(); itr2!= (*itr)["centre"].end(); itr2++){
                if(i<3){
                    centre[i++] = (*itr2).get<float>();
                }
            }
            currentLight.centre = centre;
        } else if (type=="area") {
            Eigen::Vector3f p1(0,0,0);
            Eigen::Vector3f p2(0,0,0);
            Eigen::Vector3f p3(0,0,0);
            Eigen::Vector3f p4(0,0,0);
            int i = 0;
            for (auto itr2 =(*itr)["p1"].begin(); itr2!= (*itr)["p1"].end(); itr2++){
                if(i<3){
                    p1[i++] = (*itr2).get<float>();
                }
            }
            i = 0;
            for (auto itr2 =(*itr)["p2"].begin(); itr2!= (*itr)["p2"].end(); itr2++){
                if(i<3){
                    p2[i++] = (*itr2).get<float>();
                }
            }
            i = 0;
            for (auto itr2 =(*itr)["p3"].begin(); itr2!= (*itr)["p3"].end(); itr2++){
                if(i<3){
                    p3[i++] = (*itr2).get<float>();
                }
            }
            i = 0;
            for (auto itr2 =(*itr)["p4"].begin(); itr2!= (*itr)["p4"].end(); itr2++){
                if(i<3){
                    p4[i++] = (*itr2).get<float>();
                }
            }
            currentLight.p1 = p1;
            currentLight.p2 = p2;
            currentLight.p3 = p3;
            currentLight.p4 = p4;
        }

        lightList->push_back(currentLight);
        ++lc;
    }
    return true;
}

bool parseOutput(json& j, vector<Output>* outputList){
    int lc = 0;
    
    // use iterators to read-in array types
    for (auto itr = j["output"].begin(); itr!= j["output"].end(); itr++){
        
        std::string filename;
        if(itr->contains("filename")){
          //  filename = static_cast<std::string>((*itr)["filename"]);
            filename = (*itr)["filename"].get<std::string>();
        } else {
            return false;
        }
        
        Output currentOutput;

        currentOutput.filename = filename;

        int size[2];
        int i = 0;
        for (auto itr2 =(*itr)["size"].begin(); itr2!= (*itr)["size"].end(); itr2++){
            if(i<2){
                //size[i++] = (*itr2).get<float>();
                currentOutput.size[i++] = (*itr2).get<float>();
            }
        }
        
        if(itr->contains("fov")){
            auto fovRadian = (*itr)["fov"].get<float>();
            //auto PI = atan(1) * 4;
            currentOutput.fov = fovRadian * PI / 180;
        }

        Eigen::Vector3f lookat(0,0,0), up(0,0,0), centre(0,0,0);
        Eigen::Vector3f ai(0,0,0), bkc(0,0,0);
        
        i = 0;
        for (auto itr2 =(*itr)["up"].begin(); itr2!= (*itr)["up"].end(); itr2++){
            if(i<3){
                up[i++] = (*itr2).get<float>();
            }
        }
        i = 0;
        for (auto itr2 =(*itr)["lookat"].begin(); itr2!= (*itr)["lookat"].end(); itr2++){
            if(i<3){
                lookat[i++] = (*itr2).get<float>();
            }
        }
        i = 0;
        for (auto itr2 =(*itr)["ai"].begin(); itr2!= (*itr)["ai"].end(); itr2++){
            if(i<3){
                ai[i++] = (*itr2).get<float>();
            }
        }
        i = 0;
        for (auto itr2 =(*itr)["bkc"].begin(); itr2!= (*itr)["bkc"].end(); itr2++){
            if(i<3){
                bkc[i++] = (*itr2).get<float>();
            }
        }
        i = 0;
        for (auto itr2 =(*itr)["centre"].begin(); itr2!= (*itr)["centre"].end(); itr2++){
            if(i<3){
                centre[i++] = (*itr2).get<float>();
            }
        }

        Eigen::Vector2i raysPP(0, 0);
        i = 0;
        for (auto itr2 =(*itr)["raysperpixel"].begin(); itr2!= (*itr)["raysperpixel"].end(); itr2++){
            if(i<2){
                raysPP[i++] = (*itr2).get<float>();
                //currentOutput.raysperpixel[i++] = (*itr2).get<float>();
            }
        }
        currentOutput.raysperpixel = raysPP;
        
        currentOutput.up = up;
        currentOutput.lookat = lookat;
        currentOutput.ai = ai;
        currentOutput.bkc = bkc;
        currentOutput.centre = centre;

        if(itr->contains("speedup")){
            currentOutput.speedup = (*itr)["speedup"].get<float>();
        }
        if(itr->contains("antialiasing")){
            currentOutput.antialiasing = (*itr)["antialiasing"].get<bool>();
        }
        if(itr->contains("twosiderender")){
            currentOutput.twosiderender = (*itr)["twosiderender"].get<bool>();
        }
        if(itr->contains("globalillum")){
            currentOutput.globalillum = (*itr)["globalillum"].get<bool>();
        }
        if(itr->contains("maxbounces")){
            currentOutput.maxbounces = (*itr)["maxbounces"].get<float>();
        }
        if(itr->contains("probterminate")){
            currentOutput.probterminate = (*itr)["probterminate"].get<float>();
        }
        // Similarly to the centre array you need to read the lookat and up
        //Maybe create a separate functiomn to read arrays - ythey are pretty common
        
        // I am retrieving the field of view
        // this is mandatory field here, but if I dont check if it exists,
        // the code will throw an exception which if not caught will end the execution of yoru program
        
        outputList->push_back(currentOutput);
        ++lc;
    }
    
    return true;
}
