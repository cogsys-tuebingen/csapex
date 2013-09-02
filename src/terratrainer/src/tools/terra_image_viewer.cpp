#include <utils/LibCvTools/terra_mat.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

int main(int argc, char *argv[])
{
    std::string path = argv[1];

    if(argc == 1) {
        std::cout << "Arguments : <terrr_mat> [<zoom>]" << std::endl;
        return 1;
    }


    if(!boost::filesystem3::exists(path)) {
        std::cerr << "File does not exist '" << path << "' !" << std::endl;
        return 1;
    }

    int zoom = 1;
    if(argc > 2) {
        zoom = atoi(argv[2]);
    }


    TerraMat mat;
    mat.read(path);
    cv::Mat rgb = mat.getFavoritesRGB();
    cv::Mat render(rgb.rows * zoom , rgb.cols * zoom , CV_8UC3, cv::Scalar::all(0));
    for(int i = 0 ; i < rgb.rows ; i++) {
        for(int j = 0 ; j < rgb.rows ; j++) {
            cv::Rect rect(i * zoom, j * zoom , zoom, zoom);
            cv::Vec3b  rgb_val = rgb.at<cv::Vec3b>(i,j);
            cv::Scalar color(rgb_val[2] , rgb_val[1], rgb_val[0]);

            cv::rectangle(render, rect, color, CV_FILLED);
        }

    }


    cv::imshow("Generation", mat.getFavoritesRGB());
    cv::waitKey(0);

    return 0;
}

