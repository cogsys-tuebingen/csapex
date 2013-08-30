#include "terra_mat.h"
#include <sstream>

int main(int argc, char *argv[])
{
    cv::Mat  test_float(40, 40, CV_32FC3, cv::Scalar::all(0));
    TerraMat test_terra(test_float);
    std::vector<cv::Vec3b> colors;
    colors.push_back(cv::Vec3b(0, 255, 0));
    colors.push_back(cv::Vec3b(0, 0, 255));
    colors.push_back(cv::Vec3b(0, 255, 255));

    for(int i = 0 ; i < 3 ; i++) {
        cv::Vec3f val;
        val[i] = 1.f;
        test_terra.at<cv::Vec3f>(i*10,i*10) = val;
        std::stringstream name;
        name << " name " << i;

        TerrainClass cl(i, name.str(), colors[i]);
        test_terra.addLegendEntry(cl);
    }

    cv::Mat render = test_terra.getFavoritesRGB();
    cv::imshow("testterra", render);
    cv::waitKey(0);

    test_terra.write("/tmp/test_terra_mat.yaml");

    TerraMat test_terra2;
    test_terra2.read("/tmp/test_terra_mat.yaml");
    std::map<uchar, uchar>        mapping = test_terra2.getMapping();
    std::map<uchar, TerrainClass> legend = test_terra2.getLegend();

    std::cout << "==== MAPPING =======================" << std::endl;
    for(std::map<uchar, uchar>::iterator it = mapping.begin() ; it != mapping.end() ; it++) {
        std::cout << (int) it->first << " " << (int) it->second << std::endl;
    }

    std::cout << "==== LEGEND ========================" << std::endl;
    for(std::map<uchar, TerrainClass>::iterator it = legend.begin() ; it != legend.end() ; it++) {
        std::cout << (int) it->first << " " << (int) it->second.id << std::endl;
        std::cout << it->second.name << std::endl;
        std::cout << "["
                  << (int) it->second.color[0] << " "
                  << (int) it->second.color[1] << " "
                  << (int) it->second.color[2] << "]" << std::endl;
    }

    std::cout << "==== MATRIX ========================" << std::endl;
    for(int i = 0 ; i < 3 ; i++) {
        cv::Vec3f val;
        val = test_terra.at<cv::Vec3f>(i*10,i*10);
        std::cout << val[i] << std::endl;
    }

    render = test_terra2.getFavoritesRGB();
    cv::imshow("testterra", render);
    cv::waitKey(0);


    return 0;
}

