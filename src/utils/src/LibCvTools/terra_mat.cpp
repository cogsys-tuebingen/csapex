#include "terra_mat.h"

TerraMat::TerraMat()
    :
      channels_(0)
{
}

TerraMat::TerraMat(const cv::Mat &terra_mat) :
    channels_(terra_mat.channels()),
    step_(terra_mat.step / terra_mat.elemSize1()),
    terra_mat_(terra_mat)
{
    for(uchar i = 0 ; i < channels_ ; ++i)
        mapping_.insert(std::make_pair(i,i));
}

TerraMat::TerraMat(const cv::Mat &terra_mat, const std::map<int, int> &mapping) :
    channels_(terra_mat.channels()),
    step_(terra_mat.step / terra_mat.elemSize1()),
    mapping_(mapping),
    terra_mat_(terra_mat)
{
}

void TerraMat::setMatrix(const cv::Mat &terra_mat)
{
    channels_   = terra_mat.channels();
    step_       = terra_mat.step / terra_mat.elemSize1();
    terra_mat_  = terra_mat;
    for(uchar i = 0 ; i < channels_ ; ++i)
        mapping_.insert(std::make_pair(i,i));

}

void TerraMat::setMatrix(const cv::Mat &terra_mat, const std::map<int, int> &mapping)
{
    channels_   = terra_mat.channels();
    step_       = terra_mat.step / terra_mat.elemSize1();
    terra_mat_  = terra_mat;
    mapping_    = mapping;
}

cv::Mat TerraMat::getMatrix() const
{
    return terra_mat_.clone();
}

void TerraMat::setMapping(const std::map<int, int> &mapping)
{
    if(mapping.size() != channels_) {
        std::cerr << "Setting that mapping is impossible due to difference to amount of channels!" << std::endl;
        return;
    }

    mapping_ = mapping;
}

std::map<int, int> TerraMat::getMapping() const
{
    return mapping_;
}

void TerraMat::addLegendEntry(TerrainClass terrainClass)
{
    if (legend_.size() < channels_)
        legend_.insert(std::make_pair(terrainClass.id, terrainClass));
    else
        std::cerr << "noOfTerrainClasses exceeded" << std::endl;
}

std::map<int, TerrainClass> TerraMat::getLegend() const
{
    return legend_;
}

void TerraMat::read(const std::string &filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    readMapping(fs);
    readLegend(fs);
    readMatrix(fs);
    channels_   = terra_mat_.channels();
    step_       = terra_mat_.step / terra_mat_.elemSize1();
}

void TerraMat::write(const std::string &filename) const {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    writeMapping(fs);
    writeLegend(fs);
    writeMatrix(fs);
}

// exports an uchar image, each pixel containing the id of the favorite terrain class
cv::Mat TerraMat::getFavorites()
{
    cv::Mat result(terra_mat_.rows, terra_mat_.cols, CV_8UC1, cv::Scalar::all(0));

    float* data = (float*) terra_mat_.data;
    for (int i = 0; i < terra_mat_.rows; ++i)
        for (int j = 0; j < terra_mat_.cols; ++j) {

            // get terrain class with maximum probability
            int id = 0;
            float maxProb = 0.0f;
            /// step[1] == elemsize()
            int pixel_pos = step_ * i + j * channels_;
            for (int k = 0; k < channels_ ; k++) {
                float prob = data[pixel_pos + k];
                if(prob >= maxProb) {
                    maxProb = prob;
                    id = mapping_[k];
                }
            }
            result.at<uchar>(i,j) = id;
        }

    return result;
}

// exports an rgb image showing the color of the favorite terrain class in each pixel
cv::Mat TerraMat::getFavoritesBGR() {
    cv::Mat result(terra_mat_.rows, terra_mat_.cols, CV_8UC3, cv::Scalar::all(0));
    cv::Mat favorites = getFavorites();

    if(channels_ > legend_.size()) {
        std::cerr << "You have to add a legend with minimum size of channel amount!" << std::endl;
        return cv::Mat();
    }

    for (int i = 0; i < terra_mat_.rows; ++i) {
        for (int j = 0; j < terra_mat_.cols; ++j) {
            result.at<cv::Vec3b>(i,j) = legend_[(int) favorites.at<uchar>(i,j)].color;
        }
    }
    return result;
}

// exports an rgb image showing the weighted mean color of the terrain class in each pixel
cv::Mat TerraMat::getMeanBGR() {
    cv::Mat result(terra_mat_.rows, terra_mat_.cols, CV_8UC3, cv::Scalar::all(0));

    if(channels_ > legend_.size()) {
        std::cerr << "You have to add a legend with minimum size of channel amount!" << std::endl;
        return cv::Mat();
    }

    float* data = (float*) terra_mat_.data;
    for (int i = 0; i < terra_mat_.rows; ++i)
        for (int j = 0; j < terra_mat_.cols; ++j) {

            // get mean color
            /// step[1] == elemsize()
            int pixel_pos = step_ * i + j * channels_;
            cv::Vec3b meanColor;
            for (int k = 0; k < channels_ ; k++) {
                float prob = data[pixel_pos + k];
                meanColor += prob*legend_[k].color;
            }
            result.at<cv::Vec3b>(i,j) = meanColor;
        }

    return result;
}

// exports a hybrid rgb image of getMeanBGR and getFavoritesBGR 
cv::Mat TerraMat::getBGR() {
    cv::Mat a = getMeanBGR();
    cv::Mat b = getFavoritesBGR();

    return 0.5*a + 0.5*b;
}

TerraMat::operator cv::Mat()
{
    return terra_mat_;
}

TerraMat::operator const cv::Mat&()
{
    return terra_mat_;
}

TerraMat::operator cv::Mat &()
{
    return terra_mat_;
}

void TerraMat::writeMapping(cv::FileStorage &fs) const
{
    fs << "mapping" << "[:";
    for(std::map<int, int>::const_iterator it = mapping_.begin() ; it != mapping_.end() ; it++)
        fs << it->first << it->second;
    fs << "]";
}

void TerraMat::readMapping(const cv::FileStorage &fs)
{
    mapping_.clear();
    cv::FileNode mapping = fs["mapping"];
    for(cv::FileNodeIterator it = mapping.begin() ; it != mapping.end() ; it++) {
        std::pair<uchar, uchar> entry;
        *it >> entry.first;
        it++;
        *it >> entry.second;
        mapping_.insert(entry);
    }
}

void TerraMat::writeLegend(cv::FileStorage &fs) const
{
    fs << "legend" << "[";
    for(std::map<int,TerrainClass>::const_iterator it = legend_.begin() ; it != legend_.end() ; it++) {
        it->second.write(fs);
    }
    fs << "]";
}

void TerraMat::readLegend(const cv::FileStorage &store)
{
    legend_.clear();
    cv::FileNode legend = store["legend"];
    for(cv::FileNodeIterator it = legend.begin() ; it != legend.end() ; it++) {
        std::pair<uchar, TerrainClass> entry;
        entry.second.read(*it);
        entry.first = entry.second.id;
        legend_.insert(entry);
    }
}

void TerraMat::writeMatrix(cv::FileStorage &fs) const
{
    std::vector<cv::Mat> terra_mat_channels;
    cv::split(terra_mat_, terra_mat_channels);
    fs << "terra_mat" << "[";
    for(std::vector<cv::Mat>::iterator it = terra_mat_channels.begin() ; it != terra_mat_channels.end() ; it++) {
//        /// TESTING
//        cv::fastNlMeansDenoising(*it, *it);
//        /// TESTING
        fs << *it;

    }
    fs << "]";
}

void TerraMat::readMatrix(const cv::FileStorage &fs)
{
    fs["terramat"] >> terra_mat_;
    cv::FileNode terra_mat = fs["terra_mat"];
    std::vector<cv::Mat> terra_mat_channels;

    for(cv::FileNodeIterator it = terra_mat.begin() ; it != terra_mat.end() ; it++) {
        cv::Mat channel;
        *it >> channel;
        terra_mat_channels.push_back(channel);
    }

    cv::merge(terra_mat_channels, terra_mat_);
}
