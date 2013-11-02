#ifndef EXTRACTOR_H
#define EXTRACTOR_H

/// PROJECT
#include <utils_param/parameter_provider.h>

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

        virtual void init(Extractor*, const param::ParameterProvider&) = 0;
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
     * @brief extractKeypoints extract keypoints from an image
     * @param frame the image to extract from
     * @param mask_roi mask to use when extracting
     * @param keypoints output variable for found keypoints
     */
    void extractKeypoints(const cv::Mat& frame, const cv::Mat& mask_roi, std::vector<cv::KeyPoint> &keypoints) const;

    /**
     * @brief extractDescriptors extract keypoints and descriptors from an image
     * @param frame the image to extract from
     * @param keypoints input variable for keypoints
     * @param descriptors output variable for appropriate descriptors
     */
    void extractDescriptors(const cv::Mat& frame, std::vector<cv::KeyPoint> &keypoints, cv::Mat& descriptors) const;

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

public:
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;

    std::string keypoint;
    std::string descriptor;

    bool is_binary;
    bool has_orientation;
};

#endif // EXTRACTOR_H
