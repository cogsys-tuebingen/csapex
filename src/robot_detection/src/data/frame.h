#ifndef FRAME_H
#define FRAME_H

/// COMPONENT
#include "matchable.h"

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

/**
 * @brief The Frame class represents the current camera image
 */
class Frame : public Matchable
{
    friend class MatchablePose;
    friend class FrameIO;
    friend class Painter;

public:
    typedef boost::shared_ptr<Frame> Ptr;

    static Ptr NULL_FRAME;

    typedef boost::function<void(const cv::Mat&, const cv::Mat&, std::vector<cv::KeyPoint>&, cv::Mat&)> ExtractorFunction;
    typedef boost::function<void(Frame*)> FilterFunction;

public:
    /**
     * @brief Frame
     * @param img
     */
    Frame(const cv::Mat& img);

    /**
     * @brief Frame
     * @param img
     * @param mask
     */
    Frame(const cv::Mat& img, const cv::Mat& mask);

    /**
     * @brief ~Frame
     */
    virtual ~Frame();

    /**
     * @brief get_dimensions
     * @return the width and height of this matchable
     */
    virtual cv::Rect getDimensions() const;

    /**
     * @brief extractFeatures extract features in the current roi
     * @param extractor Extractor callback
     */
    void extractFeatures(ExtractorFunction extractor);

    /**
     * @brief filterNegativeMatches remove keypoints that match a negative sample
     */
    void filterNegativeMatches(FilterFunction matcher);

    /**
     * @brief getScaledCopy makes a deep, scaled copy
     * @param scale factor to scale by
     * @return new Frame, responsibility for deleting given to caller
     */
    Frame::Ptr getScaledCopy(double scale) const;

    /**
     * @brief isValid
     * @return true, iff the current frame contains a valid image
     */
    bool isValid() const;

    /**
     * @brief isRoiFullFrame
     * @return true, iff the current roi's extent is the whole image
     */
    bool isRoiFullFrame() const;

    /**
     * @brief getWidth
     * @return width of the image
     */
    int getWidth() const;

    /**
     * @brief getHeight
     * @return height of the image
     */
    int getHeight() const;

    /**
     * @brief getRoiWidth
     * @return width of the region of interest
     */
    int getRoiWidth() const;

    /**
     * @brief getRoiHeight
     * @return height of the region of interest
     */
    int getRoiHeight() const;

    /**
     * @brief setRoi Setter for the region of interest
     * @param roi
     */
    void setRoi(cv::Rect roi = cv::Rect(0,0,-1,-1));

    /**
     * @brief getImage Accessor
     * @return the wrapped image
     */
    cv::Mat getImage();

    /**
     * @brief getImageRoi Accessor
     * @return the wrapped image's roi
     */
    cv::Mat getImageRoi();


    /**
     * @brief getDebugImage Accessor for the debug image
     * @note finalizeDebugImage must be called to get recent drawings
     */
    cv::Mat getDebugImage();

    /**
     * @brief finalizeDebugImage copies the temporary debug image to the buffer
     */
    void finalizeDebugImage();

private:

    void setImage(const cv::Mat& img);
    void setImage(const cv::Mat& img, const cv::Mat& mask);

    void init();

private:
    mutable boost::recursive_mutex mutex;

    cv::Mat image_raw;
    cv::Mat mask_raw;

    cv::Mat image_roi;
    cv::Mat mask_roi;

    cv::Rect roi;

    cv::Mat debug_img_tmp;
    cv::Mat debug_img_tmp_roi;
    cv::Mat debug_img;

    // debug
    int deleted;
};

#endif // FRAME_H
