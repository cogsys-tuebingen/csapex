#ifndef FRAME_BUFFER_H
#define FRAME_BUFFER_H

/// PROJECT
#include <common/global.hpp>

/// SYSTEM
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief The FrameBuffer class buffers serveral images in a threadsafe manor
 */
class FrameBuffer
{
public:
    /**
     * @brief FrameBuffer
     */
    FrameBuffer();

    /**
     * @brief setImage replace an image
     * @param idx index of extra to use
     * @param name name for the extra
     * @param image image to use
     */
    static void setImage(int idx, const std::string& name, cv::Mat& image);

    /**
     * @brief setImageAside merge a custom image aside of the debug image and
     *        put it into an extra slot
     * @param idx index of extra to use
     * @param name name for the extra
     * @param left left part of the new image
     * @param image right part of the new image
     */
    static void setImageAside(int idx, const std::string& name, cv::Mat left, cv::Mat& image);

    /**
     * @brief getImage Accessor
     * @param idx index of extra to use
     * @param out Output: Image
     */
    static cv::Mat getImage(int idx);

    /**
     * @brief getImageCount
     * @return the number of extra slots defined
     */
    static int getImageCount();

    /**
     * @brief getName Accessor
     * @param index
     * @return the name of the i-th extra slot
     */
    static std::string getName(int index);

private:
    static boost::recursive_mutex extra_mutex;

    static std::vector<cv::Mat> extra_debug_img;
    static std::vector<std::string> extra_name;
};

#endif // FRAME_BUFFER_H
