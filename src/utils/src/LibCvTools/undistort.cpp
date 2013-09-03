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
int parse(  int argc, char **argv, std::string &param_path, std::string &input_path, std::string &output_path, int &margin_x, int &margin_y)
{
    if(argc < 4) {
        msg_out::error("Not enough arguments supplied!");
        msg_out::error("Syntax: undistort <input image> <output image> <camera_parameters> [margin[x]] [margin[y]]");
        return 1;
    }

    param_path  = argv[3];
    input_path  = argv[1];
    output_path = argv[2];

    if(argc > 4) {
        std::stringstream conv;
        conv << argv[4];
        conv >> margin_x;
    }

    if(argc > 5) {
        std::stringstream conv;
        conv << argv[5];
        conv >> margin_y;
    }

    return 0;
}
}

int main(int argc, char **argv)
{
    std::string     param_path("");
    std::string     input_image_path("");
    std::string     output_image_path("");
    int             margin_x = 0;
    int             margin_y = 0;

    if(input::parse(argc, argv, param_path, input_image_path, output_image_path, margin_x, margin_y) == 1){
        return 1;
    }

    cv::FileStorage fs(param_path, cv::FileStorage::READ);
    cv::Mat         img = cv::imread(input_image_path);

    if(!fs.isOpened()) {
        msg_out::error("Couldn't open parameters file '" + param_path + "' !");
        return 1;
    }

    if(img.empty()) {
        msg_out::error("Couldn't load image flie '" + input_image_path, output_image_path + "' !");
        return 1;
    }

    cv::Mat camera_matrix, distorition_coeffs;
    try {
        fs["intrinsics"] >> camera_matrix;
    } catch(cv::Exception e) {
        msg_out::error("Entry 'intrinsics' not found in '" + param_path +"' !");
        fs.release();
        return 1;
    }
    try {
        fs["distortion"] >> distorition_coeffs;
    }      catch(cv::Exception e) {
        msg_out::error("Entry 'distortion' not found in '" + param_path + "' !");
        fs.release();
        return 1;
    }

    if(margin_x != 0) {

        if(margin_y == 0)
            margin_y = margin_x;

        margin_x *= 4;
        margin_y *= 3;

        cv::Mat img_scaled(img.rows + 2 * margin_y, img.cols + 2 * margin_x, img.type());
        cv::Mat roi(img_scaled, cv::Rect(margin_x, margin_y, img.cols, img.rows));
        img.copyTo(roi);
        img = img_scaled;
    }

    Undistorter u(camera_matrix, distorition_coeffs);
    u.reset_map(cv::Size(img.cols, img.rows), margin_x, margin_y);
    u.undistort(img, img);
    cv::imwrite(output_image_path, img);

    return 0;
}
