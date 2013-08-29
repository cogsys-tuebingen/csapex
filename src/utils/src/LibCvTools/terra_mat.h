#ifndef TERRAMAT
#define TERRAMAT

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct TerrainClass {
    TerrainClass(uchar id, std::string name, cv::Vec3b color)
        :
          id(id),
          name(name),
          color(color) {
    }

    TerrainClass()
    {
    }

    uchar       id;
    std::string name;
    cv::Vec3b   color;
};


class TerraMat
{
public:
    TerraMat();
    TerraMat(const cv::Mat &terra_mat);
    TerraMat(const cv::Mat &terra_mat, const std::map<uchar, uchar> &mapping);

    void addTerrainClass(TerrainClass terrainClass);

    void read(std::string filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        fs["terramat"] >> terra_mat_;
    }

    void write(std::string filename) {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        fs << "terramat" << terra_mat_;
    }

    // exports an uchar image, each pixel containing the id of the favorite terrain class
    cv::Mat getFavorites();

    // exports an rgb image showing the color of the favorite terrain class in each pixel
    cv::Mat getFavoritesRGB();

    operator cv::Mat();

    template<typename _Tp>
    _Tp& at(int i, int j)
    {
        return terra_mat_.at<_Tp>(i,j);
    }

private:
    const int                   channels_;
    int                         step_;
    std::map<uchar,uchar>       mapping_;
    std::map<uchar,TerrainClass>legend_;
    cv::Mat                     terra_mat_;
};


#endif
