/// HEADER
#include "masked_image_handler.h"

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <ros/console.h>

using namespace background_subtraction;

MaskedImageHandler::MaskedImageHandler()
{
}

background_subtraction::MaskedImage MaskedImageHandler::wrap(const std_msgs::Header& header,
        const sensor_msgs::ImageConstPtr& img,
        const cv::Mat& mask) const
{
    cv_bridge::CvImage mask_converter;
    mask_converter.encoding = "mono8";
    mask_converter.header = header;
    mask_converter.image = mask;

    background_subtraction::MaskedImage masked;
    masked.header = header;
    masked.img = *img;
    masked.mask = *mask_converter.toImageMsg();

    return masked;
}

void MaskedImageHandler::import(const sensor_msgs::Image& img, const std::string& enc, cv::Mat& result) const
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, enc);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    result = cv_ptr->image;
}

void MaskedImageHandler::import(const sensor_msgs::ImageConstPtr& img, const std::string& enc, cv::Mat& result) const
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, enc);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    result = cv_ptr->image;
}

void MaskedImageHandler::import(const background_subtraction::MaskedImageConstPtr& msg, cv::Mat& frame, cv::Mat& mask) const
{
    import(msg->img, sensor_msgs::image_encodings::BGR8, frame);
    import(msg->mask, sensor_msgs::image_encodings::MONO8, mask);
}

cv::Rect MaskedImageHandler::findRoi(const cv::Mat mask) const
{
    cv::Mat tmp;
    mask.copyTo(tmp);
    std::vector< std::vector< cv::Point> > contours;
    cv::findContours(tmp, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    cv::Rect best;
    double best_area = 0;
    for(std::vector< std::vector< cv::Point > >::iterator it = contours.begin(); it != contours.end(); ++it) {
        int area = cv::contourArea(*it);
        if(area > best_area) {
            best_area = area;
            best = cv::boundingRect(*it);
        }
    }

    if(best.width == 0 || best.height == 0) {
        return cv::Rect(0, 0, mask.cols, mask.rows);
    }

    return best;


    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();
    int max_x = std::numeric_limits<int>::min();
    int max_y = std::numeric_limits<int>::min();
    int w = mask.cols;
    int h = mask.rows;
    int i = 0;

    unsigned ws = w * mask.channels();
    for(int y = 0; y < h; ++y) {
        for(int x = 0; x < w; ++x) {
            unsigned idx = y * ws + x;

            if(mask.data[idx] > 127) {
                if(x < min_x) min_x = x;
                if(x > max_x) max_x = x;
                if(y < min_y) min_y = y;
                if(y > max_y) max_y = y;

                i++;
            }
        }
    }

    if(i == 0) {
        return cv::Rect (0, 0, mask.cols, mask.rows);
    } else {
        return cv::Rect (min_x,min_y, (max_x-min_x), (max_y-min_y));
    }
}
