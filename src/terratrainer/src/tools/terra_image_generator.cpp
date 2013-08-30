#include "terra_image_generator.h"
#include "terra_mat.h"
#include <utils/LibCvTools/terra_mat_generation.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>

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

        /// READ CLASSIFIE
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

int main(int argc, char *argv[])
{
    TerraImageGenerator t;

    if(argc != 4) {
        std::cerr << "Not engough arguments!" << std::endl;
        std::cerr << "Arguments : <classifier> <image> <cellsize>" << std::endl;
        return 1;
    }

    std::ifstream in(argv[1]);
    if(in.is_open()) {
        t.read(in);
    } else {
        std::cerr << "Couldn't open classifier file '" << argv[1] << "'!" << std::endl;
        return 1;
    }

    cv::Mat tmp = cv::imread(argv[2]);
    if(tmp.empty()) {
        std::cerr << "Couldn't open classifier file '" << argv[2] << "'!" << std::endl;
        return 1;
    }
    t.setImage(tmp);
    t.setCellSize(atoi(argv[3]));
    t.run();
    t.write("result.yml");

    TerraMat mat = t.getTerraMat();
    cv::imshow("Generation", mat.getFavoritesRGB());
    cv::waitKey(0);

    return 0;
}
