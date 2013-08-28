#ifndef CV_DECOMPOSITION_CLASSIFIER_HPP
#define CV_DECOMPOSITION_CLASSIFIER_HPP
#include <opencv2/core/core.hpp>
#include "randomforest.h"
#include "feature_extractor.h"
#include "pattern_extractor.h"
#include "extractor_params.h"

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

class TerraDecomClassifierFeature : public DecompositionClassifier
{
    ///
public :
    TerraDecomClassifierFeature(const float _threshold, RandomForest *_classifier, cv_extraction::FeatureExtractor *_extractor,
                           const cv_extraction::KeypointParams &_key, const cv_extraction::ExtractorParams &_params) :
        classifier(_classifier),
        extractor(_extractor),
        threshold(_threshold),
        last_prob(0.0),
        last_id(-1),
        key(_key),
        params(_params)
    {
    }

    virtual ~TerraDecomClassifierFeature()
    {
    }

    bool classify(const cv::Rect &roi)
    {
        /// EXTRACT
        cv::Mat descriptors;
        extractor->extract(image, roi, descriptors);

        /// PREDICT
        if(descriptors.empty()) {
            return false;
        }

        if(descriptors.type() != CV_32FC1) {
            descriptors.convertTo(descriptors, CV_32FC1);
        }

        if(descriptors.rows > 1) {
            classifier->predictClassProbMultiSample(descriptors, last_id, last_prob);
        } else {
            classifier->predictClassProb(descriptors, last_id, last_prob);
        }

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
    RandomForest                    *classifier;
    cv_extraction::FeatureExtractor *extractor;
    float                           threshold;
    cv::Mat                         image;
    float                           last_prob;
    int                             last_id;
    cv_extraction::KeypointParams   key;
    cv_extraction::ExtractorParams  params;
};

class TerraDecomClassifierPattern : public DecompositionClassifier
{
    ///
public :
    TerraDecomClassifierPattern(const float _threshold, RandomForest *_classifier, cv_extraction::PatternExtractor *_extractor) :
        classifier(_classifier),
        extractor(_extractor),
        threshold(_threshold),
        last_prob(0.0),
        last_id(-1)
    {
    }

    virtual ~TerraDecomClassifierPattern()
    {
    }

    bool classify(const cv::Rect &roi)
    {

        cv::Mat descriptors;
        cv::Mat img_roi(image, roi);
        extractor->extract(img_roi, descriptors);
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
    RandomForest                       *classifier;
    cv_extraction::PatternExtractor    *extractor;
    float                               threshold;
    cv::Mat                             image;
    float                               last_prob;
    int                                 last_id;
};

#endif // DECOMPOSITION_CLASSIFIER_HPP
