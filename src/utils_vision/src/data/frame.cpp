/// HEADER
#include "frame.h"

/// PROJECT
#include <data/frame_io.h>

/// SYSTEM
#include <boost/thread/locks.hpp>
#include <opencv2/opencv.hpp>

Frame::Ptr Frame::NULL_FRAME(new Frame(cv::Mat(1, 1, 0)));

Frame::Frame(const cv::Mat& img)
{
    setImage(img);
    init();
}

Frame::Frame(const cv::Mat& img, const cv::Mat& mask)
{
    setImage(img, mask);
    init();
}

cv::Rect Frame::getDimensions() const
{
    return cv::Rect(0, 0, image_raw.cols, image_raw.rows);
}

void Frame::init()
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    orientation = 0;
    distance = -1;

    deleted = -1;
}

Frame::~Frame()
{
    {
        boost::recursive_mutex::scoped_lock lock(mutex);
        deleted = 1;
    }
}

void Frame::extractFeatures(ExtractorFunction extractor)
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    assert(!image_roi.empty());

    extractor(image_roi, mask_roi, keypoints, descriptors);
}

void Frame::filterNegativeMatches(FilterFunction filter)
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    filter(this);
}

Frame::Ptr Frame::getScaledCopy(double scale) const
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    cv::Mat img;
    cv::resize(image_roi, img, cv::Size(), scale, scale, CV_INTER_LINEAR);

    Frame::Ptr tmp = FrameIO::convert(img);

    tmp->orientation = orientation;
    tmp->distance = distance / scale; // TODO real perspective transformation

    return tmp;
}

bool Frame::isValid() const
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    return !image_roi.empty() && deleted == -1;
}

bool Frame::isRoiFullFrame() const
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    return std::abs(roi.width - image_raw.cols) < 4 && std::abs(roi.height - image_raw.rows) < 4;
}

int Frame::getWidth() const
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    return image_roi.cols;
}

int Frame::getHeight() const
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    return image_roi.rows;
}

int Frame::getRoiWidth() const
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    return roi.width;
}

int Frame::getRoiHeight() const
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    return roi.height;
}

void Frame::setRoi(cv::Rect roi)
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    assert(image_raw.cols > 0 && image_raw.rows > 0);

    if(roi.width == -1 && roi.height == -1) {
        roi = cv::Rect(0, 0, image_raw.cols, image_raw.rows);
    }

    this->roi = roi;
    image_roi = cv::Mat(image_raw, roi);
    if(mask_raw.cols > 0) {
        mask_roi = cv::Mat(mask_raw, roi);
    }

    debug_img_tmp_roi = cv::Mat(debug_img_tmp, roi);
}

void Frame::setImage(const cv::Mat& img)
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    image_raw = img;
    image_raw.copyTo(debug_img_tmp);

    setRoi();
}


void Frame::setImage(const cv::Mat& img, const cv::Mat& mask)
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    mask_raw = mask;

    setImage(img);
}

cv::Mat Frame::getImage()
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    cv::Mat tmp;
    image_raw.copyTo(tmp);
    return tmp;
}

cv::Mat Frame::getImageRoi()
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    cv::Mat tmp;
    image_roi.copyTo(tmp);
    return tmp;
}

cv::Mat Frame::getDebugImage()
{
    // TODO: crashes sometimes... does the recursive_mutex work at all?
    boost::recursive_mutex::scoped_lock lock(mutex);

    cv::Mat out;
    debug_img.copyTo(out);
    return out;
}

void Frame::finalizeDebugImage()
{
    boost::recursive_mutex::scoped_lock lock(mutex);

    if(!debug_img_tmp.empty()) {
        debug_img_tmp.copyTo(debug_img);
    }
}
