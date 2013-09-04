#include <opencv2/opencv.hpp>
#include "messages.hpp"
#include "undistorter.h"
#include <sstream>

/**
 * @author Hanten, Richard
 * @brief
 * Undistort images using a therefore calculated camera matrix. For avoid the loss of image data
 * due to scaling, it is possible to use a margin.
 */

namespace input {
int parse(  int argc, char **argv, std::string &input_path, std::string &output_path, int &width, int &height, double &scale)
{
    if(argc < 4) {
        msg_out::error("Not enough arguments supplied!");
        msg_out::error("Syntax: undistort <input image> <output image> <width> <height>");
        return 1;
    }

    width = 0;
    height = 0;

    input_path  = argv[1];
    output_path = argv[2];

    if(argc < 5) {
        scale = atof(argv[3]);
    } else {
        width       = atoi(argv[3]);
        height      = atoi(argv[4]);
    }
    return 0;
}
}

int main(int argc, char **argv)
{
    std::string     input_image_path("");
    std::string     output_image_path("");
    int             width  = 0;
    int             height = 0;
    double          scale  = 1;

    if(input::parse(argc, argv, input_image_path, output_image_path, width, height, scale) == 1){
        return 1;
    }

    cv::Mat         img = cv::imread(input_image_path);
    cv::Mat         out;

    if(width == 0 && height == 0) {
        width  = scale * img.cols;
        height = scale * img.rows;
    }

    if(img.empty()) {
        msg_out::error("Couldn't load image file '" + input_image_path, output_image_path + "' !");
        return 1;
    }

    if(width > 0 && height > 0) {
        cv::resize(img, out, cv::Size(width, height), cv::INTER_NEAREST);
//        cv::GaussianBlur(out, out, cv::Size(0, 0), 3);
        cv::addWeighted(out, 1.5, out, -0.5, 0, out);
    }
    cv::imwrite(output_image_path, out);

    return 0;
}

