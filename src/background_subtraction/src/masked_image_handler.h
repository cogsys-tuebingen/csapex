#ifndef MASKED_IMAGE_HANDLER_H
#define MASKED_IMAGE_HANDLER_H

/// SYSTEM
#include <sensor_msgs/Image.h>
#include <background_subtraction/MaskedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace background_subtraction {

/**
 * @brief The MaskedImageHandler class helps at using masked images
 */
class MaskedImageHandler
{
public:
    /**
     * @brief MaskedImageHandler
     */
    MaskedImageHandler();

    /**
     * @brief wrap combines a ROS image and an OpenCV image into a MaskedImage
     * @param header the header to use
     * @param img an ROS image
     * @param mask an OpenCV mask image
     * @return the combined image
     */
    background_subtraction::MaskedImage wrap(const std_msgs::Header &header,
                                      const sensor_msgs::ImageConstPtr &img,
                                      const cv::Mat &mask) const;

    /**
     * @brief import converts a masked image into two OpenCV images
     * @param msg input masked image
     * @param frame Output: first part of the masked image
     * @param mask Output: second part of the masked image
     */
    void import(const background_subtraction::MaskedImageConstPtr &msg, cv::Mat &frame, cv::Mat &mask) const;

    /**
     * @brief find_roi finds a minimum enclosing axis aligned rectangle for the mask
     * @param mask
     * @return roi
     */
    cv::Rect findRoi(const cv::Mat mask) const;

private:
    void import(const sensor_msgs::Image &img, const std::string &enc, cv::Mat &result) const;
    void import(const sensor_msgs::ImageConstPtr &img, const std::string &enc, cv::Mat &result) const;
};

}
#endif // MASKED_IMAGE_HANDLER_H
