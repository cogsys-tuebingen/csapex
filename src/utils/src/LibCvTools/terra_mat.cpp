#include "terra_mat.h"

TerraMat::TerraMat()
    :
      channels_(0)
{
}

TerraMat::TerraMat(const cv::Mat &terra_mat, const std::map<uchar, uchar> &mapping) :
    channels_(terra_mat.channels()),
    step_(terra_mat.step / terra_mat.elemSize1()),
    mapping_(mapping),
    terra_mat_(terra_mat)
{
}

TerraMat::TerraMat(const cv::Mat &terra_mat) :
    channels_(terra_mat.channels()),
    step_(terra_mat.step / terra_mat.elemSize1()),
    terra_mat_(terra_mat)
{
    for(uchar i = 0 ; i < channels_ ; i++)
        mapping_.insert(std::make_pair(i,i));
}

void TerraMat::addTerrainClass(TerrainClass terrainClass)
{
    if (legend_.size() < channels_)
        legend_.insert(std::make_pair(terrainClass.id, terrainClass));
    else
        std::cerr << "noOfTerrainClasses exceeded" << std::endl;
}

// exports an uchar image, each pixel containing the id of the favorite terrain class
cv::Mat TerraMat::getFavorites()
{
    cv::Mat result(terra_mat_.rows, terra_mat_.cols, CV_8UC1, cv::Scalar::all(0));

    float* data = (float*) terra_mat_.data;
    for (int i = 0; i < terra_mat_.rows; i++)
        for (int j = 0; j < terra_mat_.cols; j++) {

            // get terrain class with maximum probability
            uchar id = 0;
            float maxProb = 0.0f;
            /// step[1] == elemsize()
            int pixel_pos = step_ * i + j * channels_;
            for (int k = 0; k < channels_ ; k++) {
                float prob = data[pixel_pos + k];
                if(prob > maxProb) {
                    maxProb = prob;
                    id = mapping_[k];
                }
            }
            result.at<uchar>(i,j) = id;
        }

    return result;
}

// exports an rgb image showing the color of the favorite terrain class in each pixel
cv::Mat TerraMat::getFavoritesRGB() {
    cv::Mat result(terra_mat_.rows, terra_mat_.cols, CV_8UC3, cv::Scalar::all(0));
    cv::Mat favorites = getFavorites();

    if(channels_ > legend_.size()) {
        std::cerr << "You have to add a legend with minimum size of channel amount!" << std::endl;
        return cv::Mat();
    }

    for (int i = 0; i < terra_mat_.rows; i++)
        for (int j = 0; j < terra_mat_.cols; j++)
            result.at<cv::Vec3b>(i,j) = legend_[favorites.at<uchar>(i,j)].color;

    return result;
}

TerraMat::operator cv::Mat()
{
    return terra_mat_;
}

void TerraMat::read(std::string filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["terramat"] >> terra_mat_;
}

void TerraMat::write(std::string filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "terramat" << terra_mat_;
}
