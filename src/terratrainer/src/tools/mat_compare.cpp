#include <utils/LibCvTools/terra_mat.h>
#include <iostream>
#include <fstream>
#include <ostream>
#include <yaml-cpp/yaml.h>

int main(int argc, char *argv[])
{
    if(argc < 4) {
        std::cerr << "Arguments : <terramat ground truth> <terramat> <out_file> [<mat name>]" << std::endl;
        return 1;
    }


    TerraMat mat_gt;
    TerraMat mat_terra;
    mat_gt.read(argv[1]);
    mat_terra.read(argv[2]);

    std::string out_path = argv[3];
    std::string mat_name = "";
    if(argc > 4) {
        mat_name = argv[4];
    }

    cv::Mat val_gt;
    cv::Mat val_terra;
    cv::Mat fav_gt      = mat_gt.getFavorites(val_gt, 0.001f);
    cv::Mat fav_terra   = mat_terra.getFavorites(val_terra, 0.001f);

    int pos = 0;
    int neg = 0;
    int ign = 0;

    std::map<int, std::map<int, int> > matches;

    for(int i = 0 ; i < fav_gt.rows ; ++i) {
        for(int j = 0 ; j < fav_gt.cols ; ++j) {

            /// ONLY CHECK ON MATCHABLES
            if(val_gt.at<uchar>(i,j) == 1 && val_terra.at<uchar>(i,j) == 1) {
                int class_gt    = (int) fav_gt.at<uchar>(i,j);
                int class_terra = (int) fav_terra.at<uchar>(i,j);
                if(matches.find(class_terra) == matches.end()) {
                    matches.insert(std::make_pair(class_terra, std::map<int,int>()));
                }

                if(matches[class_terra].find(class_gt) == matches[class_terra].end()) {
                    matches[class_terra].insert(std::make_pair(class_gt, 0));
                }

                (matches[class_terra])[class_gt]++;
                if(class_gt == class_terra)
                    pos++;
                else
                    neg++;

            } else
                ign++;
        }
    }

    std::ofstream out(out_path.c_str(), std::fstream::app);
    if(!out.is_open()) {
        std::cerr << "Problem writing file!" << std::endl;
    }

    YAML::Emitter emitter;
    emitter << YAML::BeginSeq;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "NAME" << YAML::Value << mat_name;
    emitter << YAML::Key << "RESULTS" << YAML::Value;
    emitter << YAML::BeginMap;
    for(std::map<int, std::map<int, int> >::iterator it = matches.begin() ; it != matches.end() ; ++it) {
        emitter << YAML::Key << it->first << YAML::Value;
        emitter << YAML::BeginMap;
        for(std::map<int,int>::iterator in_it = it->second.begin() ; in_it != it->second.end() ; ++in_it) {
            emitter << YAML::Key << in_it->first << YAML::Value << in_it->second;
        }
        emitter << YAML::EndMap;
    }
    emitter << YAML::EndMap;
    emitter << YAML::Key << "POS" << YAML::Value << pos;
    emitter << YAML::Key << "NEG" << YAML::Value << neg;
    emitter << YAML::Key << "IGN" << YAML::Value << ign;
    emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    out << emitter.c_str();
    out << std::endl;
    out.close();

    std::cout << "pos : " << pos << " neg :" << neg << std::endl;

    return 0;
}

