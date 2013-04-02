#ifndef FRAME_IO_H
#define FRAME_IO_H

/// COMPONENT
#include "frame.h"

/**
 * @brief The FrameIO class is responsible for Frame I/O
 */
class FrameIO
{
public:
    /**
     * @brief exportFrame Save a Frame to a path
     * @param frame
     * @param path
     * @param id an additional id for naming
     */
    static void exportFrame(Frame* frame, const std::string& path, int id);

    /**
     * @brief renderSquareRoi save an image with aspect ratio 1
     * @param frame
     * @param path target file
     */
    static void renderSquareRoi(Frame* frame, const std::string& path);

    /**
     * @brief renderMasked apply the mask to the image and save the result
     * @param frame
     * @param path target file
     */
    static void renderMasked(Frame* frame, const std::string& path);

    /**
     * @brief importRaw import an image as a new frame
     * @param path path to an image file
     * @return new Frame, responsibility for deleting given to caller
     */
    static Frame::Ptr importRaw(const std::string& path);

    /**
     * @brief importCropped import a cropped image and mask with minimal area
     * @param path path to directory containing needed files
     * @param use_mask wheter to use the mask or not
     * @return new Frame, responsibility for deleting given to caller
     */
    static Frame::Ptr importCropped(const std::string& path, bool use_mask = true);

    /**
     * @brief importFullsize import an uncropped image and mask
     * @param path path to directory containing needed files
     * @param use_mask wheter to use the mask or not
     * @return new Frame, responsibility for deleting given to caller
     */
    static Frame::Ptr importFullsize(const std::string& path, bool use_mask = true);

    /**
     * @brief import covnert a OpenCV image into a Frame
     * @param img
     * @return new Frame, responsibility for deleting given to caller
     */
    static Frame::Ptr convert(const cv::Mat& img);

    /**
     * @brief import convert a masked image into a frame
     * @param msg
     * @param use_roi whether to use a region of interest based on the mask
     * @return new Frame, responsibility for deleting given to caller
     */
    static Frame::Ptr convert(const cv::Mat& img, const cv::Mat& mask, cv::Rect roi = cv::Rect());

private:
    /**
     * @brief FrameIO abstract
     */
    FrameIO();

private:
    static Frame::Ptr importSample(const std::string& path, bool full, bool use_mask = true);

    static void readInfo(Frame* frame, const std::string& path);
};

#endif // FRAME_IO_H
