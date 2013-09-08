#include "terra_mat.h"

TerraMat::TerraMat()
    :
      channels_(0),
      COLOR_INVALID(127,127,127)
{
}

TerraMat::TerraMat(const cv::Mat &terra_mat) :
    channels_(terra_mat.channels()),
    step_(terra_mat.step / terra_mat.elemSize1()),
    terra_mat_(terra_mat),
    COLOR_INVALID(127,127,127)
{
    for(int i = 0 ; i < channels_ ; ++i) {
        mapping_.insert(std::make_pair(i,i));
        reverse_mapping_.insert(std::make_pair(i,i));
    }
}

TerraMat::TerraMat(const cv::Mat &terra_mat, const std::map<int, int> &mapping) :
    channels_(terra_mat.channels()),
    step_(terra_mat.step / terra_mat.elemSize1()),
    mapping_(mapping),
    terra_mat_(terra_mat),
    COLOR_INVALID(127,127,127)
{
    calcReversMapping();
}

void TerraMat::setMatrix(const cv::Mat &terra_mat)
{
    channels_   = terra_mat.channels();
    step_       = terra_mat.step / terra_mat.elemSize1();
    terra_mat_  = terra_mat;
    for(int i = 0 ; i < channels_ ; ++i) {
        mapping_.insert(std::make_pair(i,i));
        reverse_mapping_.insert(std::make_pair(i,i));
    }
}

void TerraMat::setMatrix(const cv::Mat &terra_mat, const std::map<int, int> &mapping)
{
    channels_   = terra_mat.channels();
    step_       = terra_mat.step / terra_mat.elemSize1();
    terra_mat_  = terra_mat;
    mapping_    = mapping;
    calcReversMapping();
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
    calcReversMapping();
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

cv::Mat TerraMat::getFavorites(cv::Mat &validity, const float thresh)
{
    cv::Mat result(terra_mat_.rows, terra_mat_.cols, CV_8UC1, cv::Scalar::all(0));
    validity = cv::Mat(terra_mat_.rows, terra_mat_.cols, CV_8UC1, cv::Scalar::all(1));

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

            validity.at<uchar>(i,j) = maxProb < thresh ? 0 : 1;
            result.at<uchar>(i,j) = id;
        }

    return result;
}

int TerraMat::getChannelOfId(const int id)
{
    return reverse_mapping_[id];
}

// exports an rgb image showing the color of the favorite terrain class in each pixel
cv::Mat TerraMat::getFavoritesBGR() {
    cv::Mat result(terra_mat_.rows, terra_mat_.cols, CV_8UC3, cv::Scalar::all(0));
    cv::Mat validity;
    cv::Mat favorites = getFavorites(validity, 0.6f);

    if(channels_ > legend_.size()) {
        std::cerr << "You have to add a legend with minimum size of channel amount!" << std::endl;
        return cv::Mat();
    }

    for (int i = 0; i < terra_mat_.rows; ++i) {
        for (int j = 0; j < terra_mat_.cols; ++j) {
            int valid = (int) validity.at<uchar>(i,j);
            if(valid == 1)
                result.at<cv::Vec3b>(i,j) = legend_[(int) favorites.at<uchar>(i,j)].color;
            else
                result.at<cv::Vec3b>(i,j) = COLOR_INVALID;
        }
    }
    return result;
}

cv::Mat TerraMat::getFavoritesBGRRaw() {
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

void TerraMat::getAbsolut(TerraMat &abs)
{
    abs.step_            = step_;
    abs.mapping_         = mapping_;
    abs.reverse_mapping_ = reverse_mapping_;
    abs.channels_        = channels_;
    abs.legend_          = legend_;

    abs.terra_mat_ = cv::Mat(terra_mat_.rows, terra_mat_.cols, terra_mat_.type(), cv::Scalar::all(0));
    cv::Mat validity;
    cv::Mat favourites = getFavorites(validity, 0.01);

    float* data = (float*) abs.terra_mat_.data;
    uchar* valid = (uchar*) validity.data;
    for(int i = 0 ; i < favourites.rows ; ++i) {
        for(int j = 0 ; j < favourites.cols; ++j) {
            int channel = reverse_mapping_[(int) favourites.at<uchar>(i,j)];
            int pixel_pos = step_ * i + j * channels_;
            if(valid[validity.step * i + j] == 1)
                data[pixel_pos + channel] = 1.f;
        }
    }
}

void TerraMat::setAbsolut(const int row, const int col, const int channel)
{
    float* data = (float*) terra_mat_.data;
    for(int k = 0 ; k < channels_ ; ++k) {
        int pixel_pos = step_ * row + col * channels_;
        if(k != channel)
            data[pixel_pos + k] = 0.f;
        else
            data[pixel_pos + k] = 1.f;
    }
}

void TerraMat::setUnknown(const int row, const int col)
{
    float* data = (float*) terra_mat_.data;
    for(int k = 0 ; k < channels_ ; ++k) {
        int pixel_pos = step_ * row + col * channels_;
        data[pixel_pos + k] = 0.f;

    }
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
    calcReversMapping();
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
        //cv::fastNlMeansDenoising(*it, *it);
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
        /// TESTING
        //cv::fastNlMeansDenoising(channel, channel);
        /// TESTING
        terra_mat_channels.push_back(channel);
    }

    cv::merge(terra_mat_channels, terra_mat_);
}

void TerraMat::calcReversMapping()
{
    reverse_mapping_.clear();
    for(std::map<int,int>::iterator it = mapping_.begin() ; it != mapping_.end() ; ++it) {
        reverse_mapping_.insert(std::make_pair(it->second, it->first));
    }
}
