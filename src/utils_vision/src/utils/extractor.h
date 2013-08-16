#ifndef EXTRACTOR_H
#define EXTRACTOR_H

/// PROJECT
#include <config/config.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief The Extractor class is responsible for extracting keypoints and their descriptors
 */
class Extractor
{
    friend class ExtractorFactory;

public:
    typedef boost::shared_ptr<Extractor> Ptr;

    static const int TARGET_MAX_FEATURE_COUNT = 2500;

    /**
     * @brief The Initializer interface to initialize an Extractor
     */
    struct Initializer {
        typedef boost::shared_ptr<Initializer> Ptr;

        virtual void init(Extractor*) = 0;
    };

    /**
     * @brief The IllegalKeypointException
     */
    class IllegalKeypointException : public std::exception {};

    /**
     * @brief The IllegalDescriptorException
     */
    class IllegalDescriptorException : public std::exception {};

private:
    /**
     * @brief Extractor
     */
    Extractor(int id);

public:
    /**
     * @brief extract extract keypoints and descriptors from an image
     * @param frame the image to extract from
     * @param mask_roi mask to use when extracting
     * @param keypoints output variable for found keypoints
     * @param descriptors output variable for appropriate descriptors
     */
    void extract(const cv::Mat& frame, const cv::Mat& mask_roi, std::vector<cv::KeyPoint> &keypoints, cv::Mat& descriptors) const;

    /**
     * @brief binary
     * @return true, iff descriptors are binary
     */
    bool binary() const {
        return is_binary;
    }

    /**
     * @brief hasOrientation
     * @return true, iff keypoints have orientation
     */
    bool hasOrientation() const {
        return has_orientation;
    }

    /**
     * @brief valid
     * @return <b>true</b>, iff detector and extractor are valid
     */
    bool valid() const;

public:
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;

    std::string keypoint;
    std::string descriptor;

    bool is_binary;
    bool has_orientation;
};

#endif // EXTRACTOR_H
