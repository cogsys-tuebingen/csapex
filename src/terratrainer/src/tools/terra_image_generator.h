#ifndef TERRA_IMAGE_GENERATOR_H
#define TERRA_IMAGE_GENERATOR_H
#include <utils/LibCvTools/extractor.h>
#include <utils/LibCvTools/terra_mat.h>
#include <utils/LibCvTools/randomforest.h>

namespace YAML {
class Iterator;
}

class TerraImageGenerator
{
public:
    TerraImageGenerator();

    void read(std::ifstream &in);
    void write(const std::string &path);
    void setImage(const cv::Mat &image);
    void setCellSize(const int value);
    void run();
    TerraMat getTerraMat();

    const std::string                   FOREST_PATH;

private:
    std::vector<int>                    ids_;
    std::vector<std::string>            infos_;
    std::vector<int>                    color_index_;
    std::vector<cv::Vec3b>              colors_;
    RandomForest::Ptr                   random_forest_;
    cv_extraction::Extractor::Ptr       extractor_;
    cv_extraction::ExtractorParams::Ptr extractor_params_;
    cv_extraction::KeypointParams       keypoint_params_;
    TerraMat                            terra_mat_;
    cv::Mat                             image_;
    int                                 cell_size_;


    void readClasses(const YAML::Iterator &begin, const YAML::Iterator &end);
    void readColors(const YAML::Iterator &begin, const YAML::Iterator &end);
    void readForest(const YAML::Node &data);


};

#endif // TERRA_IMAGE_GENERATOR_H
