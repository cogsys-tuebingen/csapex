#include "terra_mat.h"

TerraMat::TerraMat(int width, int height)
    :
      Mat(zeros(height, width, CV_32FC(NO_OF_TERRAIN_CLASSES))) {
}

void TerraMat::addTerrainClass(TerrainClass terrainClass) {
    if (legend.size() < NO_OF_TERRAIN_CLASSES)
        legend.push_back(terrainClass);
    else
        std::cerr << "Maximum no. of terrain classes exceeded!" << std::endl;
}

// exports an uchar image, each pixel containing the id of the favorite terrain class
cv::Mat TerraMat::getFavorites() const {
    Mat result(zeros(this->rows, this->cols, CV_8UC1));

    for (int i = 0; i < this->rows; i++)
        for (int j = 0; j < this->cols; j++) {

            // get terrain class with maximum probability
            uchar id = 0;
            float maxProb = 0.0f;
            for (int k = 0; k < NO_OF_TERRAIN_CLASSES; k++) {
                cv::Vec<float, NO_OF_TERRAIN_CLASSES> pixel = this->at< cv::Vec<float, NO_OF_TERRAIN_CLASSES> >(i,j);
                if (pixel[k] > maxProb) {
                    maxProb = pixel[k];
                    id = k;
                }
            }
            result.at<uchar>(i,j) = id;
        }

    return result;
}

// exports an rgb image showing the color of the favorite terrain class in each pixel
cv::Mat TerraMat::getFavoritesRGB() const {
    Mat result(zeros(this->rows, this->cols, CV_8UC3));
    Mat favorites = getFavorites();

    for (int i = 0; i < this->rows; i++)
        for (int j = 0; j < this->cols; j++)
            result.at<cv::Vec3b>(i,j) = legend[favorites.at<uchar>(i,j)].color;

    return result;
}
