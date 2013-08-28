#ifndef TERRAMAT
#define TERRAMAT

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct TerrainClass {
    TerrainClass(std::string name, cv::Vec3b color)
        :
          name(name),
          color(color) {
    }

    std::string name;
    cv::Vec3b color;
};

#define NO_OF_TERRAIN_CLASSES 5

class TerraMat: public cv::Mat
{
public:
    TerraMat(int width, int height);

    void addTerrainClass(TerrainClass terrainClass);

    // exports an uchar image, each pixel containing the id of the favorite terrain class
    Mat getFavorites() const;

    // exports an rgb image showing the color of the favorite terrain class in each pixel
    Mat getFavoritesRGB() const;

    std::vector<TerrainClass> legend;
};


#endif
