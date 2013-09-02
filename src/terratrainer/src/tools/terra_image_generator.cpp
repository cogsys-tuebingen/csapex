#include "terra_image_generator.h"
#include "terra_mat.h"
#include <utils/LibCvTools/terra_mat_generation.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

using namespace cv_extraction;

TerraImageGenerator::TerraImageGenerator() :
    FOREST_PATH("/tmp/forest.yml"),
    random_forest_(new RandomForest),
    cell_size_(10)
{
}

void TerraImageGenerator::read(std::ifstream &in)
{
    try {
        YAML::Parser parser(in);
        YAML::Node   document;
        parser.GetNextDocument(document);

        /// READ CLASSIFIER
        const YAML::Node &classifier_data = document["CLASSIFIER"];
        readForest(classifier_data);

        /// READ EXTRACTOR
        Extractor::read(classifier_data, extractor_, extractor_params_, keypoint_params_);

        /// READ CLASSES
        const YAML::Node &classes = document["CLASSES"];
        readClasses(classes.begin(), classes.end());

        /// READ COLORS
        const YAML::Node &colors = document["CLASSES_PALETTE"];
        readColors(colors.begin(), colors.end());

        std::cout << "Loaded settings and classes!" << std::endl;
    } catch (YAML::Exception e) {
        std::cerr << "Problems reading preferences : '" << e.what() <<"' !" << std::endl;
    }

}

void TerraImageGenerator::write(const std::string &path)
{
    terra_mat_.write(path);
}

void TerraImageGenerator::setImage(const cv::Mat &image)
{
    image_ = image;
}

void TerraImageGenerator::setCellSize(const int value)
{
    cell_size_ = value;
}

void TerraImageGenerator::run()
{
    cv::Mat tmp;
    terra_mat_ = TerraMat();

    std::map<int,int> ids_to_channel;
    std::map<int,int> channel_to_ids;
    for(int i = 0 ; i < ids_.size() ; i++) {
        ids_to_channel.insert(std::make_pair(ids_[i], i));
        channel_to_ids.insert(std::make_pair(i, ids_[i]));
    }

    prepare_terra_mat(image_, cell_size_, ids_.size(), extractor_, random_forest_, tmp, ids_to_channel, extractor_params_->use_max_prob);
    terra_mat_.setMatrix(tmp, channel_to_ids);

    for(int i = 0 ; i < ids_.size() ; i++) {
        TerrainClass t;
        t.id    = ids_[i];
        t.color = colors_[color_index_[i]];
        t.name  = infos_[i];
        terra_mat_.addLegendEntry(t);
    }
}

TerraMat TerraImageGenerator::getTerraMat()
{
    return terra_mat_;
}


void TerraImageGenerator::readClasses(const YAML::Iterator &begin, const YAML::Iterator &end)
{
    for(YAML::Iterator it = begin ; it != end ; it++) {
        int class_id;
        int color;
        std::string info;

        (*it)["id"]    >> class_id;
        (*it)["color"] >> color;
        (*it)["info"]  >> info;

        ids_.push_back(class_id);
        color_index_.push_back(color);
        infos_.push_back(info);
    }
}

void TerraImageGenerator::readColors(const YAML::Iterator &begin, const YAML::Iterator &end)
{
    cv::Vec3b color;
    for(YAML::Iterator it = begin ; it != end ; it++) {
        int buf;
        *it >> buf;
        color[2] = (uchar) buf;
        it++;
        *it >> buf;
        color[1] = (uchar) buf;
        it++;
        *it >> buf;
        color[0] = (uchar) buf;
        colors_.push_back(color);
    }
}

void TerraImageGenerator::readForest(const YAML::Node &data)
{
    std::ofstream out(FOREST_PATH.c_str());
    if(!out.is_open()) {
        std::cerr << "Couldn't open forest file!" << std::endl;
        return;
    }

    std::string buf;
    data["DATA"] >> buf;
    out << buf;
    out.close();

    random_forest_->load(FOREST_PATH);
}

namespace input {
bool readYAML(const std::string &yaml, std::vector<std::string> &images)
{
    std::ifstream in(yaml.c_str());
    if(!in.is_open())
        return false;

    YAML::Parser p(in);
    YAML::Node document;
    p.GetNextDocument(document);
    for(YAML::Iterator it = document.begin() ; it != document.end() ; it++) {
        std::string path;
        *it >> path;
        images.push_back(path);
    }
    in.close();
    return images.size() > 0;
}

int parse(int argc, char **argv, std::string &workpath, std::string &classifier, int &cell_size, std::vector<std::string> &images)
{


    boost::regex e_imag(".*\\.(jpg|png|tiff|pgm)");
    boost::regex e_yaml(".*\\.(yaml)");
    boost::regex e_int("[0-9]*");

    if(argc == 1) {
        std::cout << "Arguments : <classifier> <cellsize> <image | images.yaml> <outpath>" << std::endl;
        return 1;
    }

    if(argc < 4) {
        std::cerr << "Not engough arguments!" << std::endl;
        std::cerr << "Arguments : <classifier> <cellsize> <image | images.yaml> <outpath>" << std::endl;
        return 1;
    }



    boost::cmatch what;
    bool validated_input = true;

    classifier = argv[1];

    if(boost::regex_match(argv[2], what, e_int) && what[0].matched) {
        cell_size = atoi(argv[2]);
    } else
        validated_input &= false;

    if(boost::regex_match(argv[3], what, e_imag) && what[0].matched) {
        images.push_back(argv[3]);
    } else if(boost::regex_match(argv[3], what, e_yaml) && what[0].matched) {
        validated_input &= readYAML(argv[3], images);
    } else
        validated_input &= false;

    if(argc == 5) {
        if(boost::filesystem3::exists(argv[4])) {
            workpath = argv[4];
        } else
            validated_input &= false;
    } else {
        boost::filesystem3::path path(boost::filesystem3::current_path());
        workpath = path.string();
    }

    if(!validated_input){
        std::cerr << "Some arguments couldn't be used! Check your input!" << std::endl;
        std::cerr << "Arguments : <classifier> <cellsize> <image | images.yaml> <outpath>" << std::endl;
        return 1;
    }
    return 0;
}
}

int main(int argc, char *argv[])
{
    std::string workpath;
    std::string classifier;
    std::vector<std::string> images;
    int cell_size = 10;

    int validation = input::parse(argc, argv, workpath, classifier, cell_size, images);

    if(validation > 0) {
        return validation;
    }

    TerraImageGenerator t;
    t.setCellSize(cell_size);
    std::ifstream in(classifier.c_str());
    if(in.is_open()) {
        t.read(in);
    } else {
        std::cerr << "Couldn't open classifier file '" << argv[1] << "'!" << std::endl;
        return 1;
    }


    for(std::vector<std::string>::iterator it = images.begin() ; it != images.end() ; it++) {
        cv::Mat tmp = cv::imread(*it);
        if(tmp.empty()) {
            std::cerr << "Unknown image file '" << *it << "'!" << std::endl;
            continue;
        }
        t.setImage(tmp);
        t.run();
        boost::filesystem3::path save_to(*it);
        t.write(workpath + save_to.filename().string() + ".yml");
    }
    return 0;
}
