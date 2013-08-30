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

    void write(cv::FileStorage &store) const
    {
        store << "{:";
        store << "id" << id;
        store << "name" << name;
        store << "color" << "[:" << color[0] << color[1] << color[2] << "]";
        store << "}";
    }

    void read(const cv::FileNode &store)
    {
        store["id"] >> id;
        store["name"] >> name;

        std::vector<uchar> value;
        store["color"] >> value;
        color = cv::Vec3b(value[0], value[1], value[2]);
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


    void setMatrix(const cv::Mat &terra_mat);
    void setMatrix(const cv::Mat &terra_mat, const std::map<uchar, uchar> &mapping);
    cv::Mat getMatrix() const;
    void setMapping(const std::map<uchar, uchar> &mapping);
    std::map<uchar, uchar> getMapping() const;
    void addLegendEntry(TerrainClass terrainClass);
    std::map<uchar, TerrainClass> getLegend() const;

    /// I / O - Persistence
    void read(const std::string &filename);
    void write(const std::string &filename) const;

    // exports an uchar image, each pixel containing the id of the favorite terrain class
    cv::Mat getFavorites();

    // exports an rgb image showing the color of the favorite terrain class in each pixel
    cv::Mat getFavoritesRGB();

    operator cv::Mat();
    operator cv::Mat&();
    operator const cv::Mat&();

    template<typename _Tp>
    _Tp& at(int i, int j)
    {
        return terra_mat_.at<_Tp>(i,j);
    }

private:
    int                         channels_;
    int                         step_;
    std::map<uchar,uchar>       mapping_;
    std::map<uchar,TerrainClass>legend_;
    cv::Mat                     terra_mat_;

    void writeMapping(cv::FileStorage &fs) const;
    void readMapping(const cv::FileStorage &fs);
    void writeLegend(cv::FileStorage &fs) const;
    void readLegend(const cv::FileStorage &fs);
};


#endif
