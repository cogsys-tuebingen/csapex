#ifndef CV_DECOMPOSITION_CLASSIFIER_HPP
#define CV_DECOMPOSITION_CLASSIFIER_HPP
#include <opencv2/core/core.hpp>
#include "randomforest.h"
#include "extractor.h"

/**
 * @brief The DecompositionClassifier class is used for classification of rectangular areas
 *        within an image, determining, wether with what algorithm, if an area should be splitted
 *        or not.
 */
/// interface
class DecompositionClassifier
{
public:
    typedef boost::shared_ptr<DecompositionClassifier> Ptr;
    /**
     * @brief Classify a region within in an image.
     * @param roi       an region of interest
     * @return      if decomposition is required
     */
    virtual bool classify(const cv::Rect &roi) = 0;
    /**
     * @brief Set the image reference to classify.
     * @param image     an image
     */
    virtual void set_image(const cv::Mat &image) = 0;
protected:
    DecompositionClassifier()
    {
    }
};

/**
 * @brief This class can be used to classify a rectangular area. In this case the classification is
 *        defined by the maximum difference of the grey values.
 */
class GreyValueClassifier : public DecompositionClassifier
{
public:
    GreyValueClassifier(const int _threshold) :
        threshold_(_threshold)
    {
    }

    virtual ~GreyValueClassifier()
    {
    }

    bool classify(const cv::Rect &roi)
    {
        double min, max;
        cv::Mat image(grey_image_, roi);
        cv::minMaxLoc(image, &min, &max);

        return (max - min) > threshold_;
    }

    void set_image(const cv::Mat &image)
    {
        cv::cvtColor(image, grey_image_, CV_BGR2GRAY);
    }

private:
    int     threshold_;
    cv::Mat grey_image_;
};

class TerraDecomClassifier : public DecompositionClassifier
{
    ///
public :
    TerraDecomClassifier(const float _threshold, RandomForest *_classifier, CVExtractor *_extractor,
                         const bool soft_crop = false, const float scale = 1.f, const float angle = 0.f) :
        classifier(_classifier),
        extractor(_extractor),
        threshold(_threshold),
        last_prob(0.0),
        last_id(-1),
        soft_crop(soft_crop),
        key_point_angle(angle),
        key_point_scale(scale)
    {
    }

    virtual ~TerraDecomClassifier()
    {
    }

    bool classify(const cv::Rect &roi)
    {
        cv::Mat descriptors;
        cv::Mat img_roi;
        if(soft_crop)
            img_roi = image;
        else
            img_roi = cv::Mat(image, roi);

        CVExtractor::KeyPoints k = extractor->prepareKeypoint(roi, soft_crop, key_point_scale, key_point_angle);
        extractor->extract(img_roi, k, descriptors);
        classifier->predictClassProb(descriptors, last_id, last_prob);

        return last_prob < threshold;

    }

    float get_prob()
    {
        return last_prob;
    }

    int get_id()
    {
        return last_id;
    }

    void set_image(const cv::Mat &_image)
    {
        image = _image;
    }

private:
    RandomForest *classifier;
    CVExtractor    *extractor;
    float         threshold;
    cv::Mat       image;
    float         last_prob;
    int           last_id;
    bool          soft_crop;
    float         key_point_angle;
    float         key_point_scale;
};
#endif // DECOMPOSITION_CLASSIFIER_HPP
