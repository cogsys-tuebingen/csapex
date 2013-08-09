#ifndef TERRA_DECOMPOSITION_CLASSIFIERS_HPP
#define TERRA_DECOMPOSITION_CLASSIFIERS_HPP
#include "decomposition_classifiers.hpp"
#include "randomforest.h"
#include "extractor.h"
class TerraDecomClassifier : public DecompositionClassifier
{
public :
    TerraDecomClassifier(const float _threshold, RandomForest *_classifier, Extractor *_extractor) :
        classifier(_classifier),
        extractor(_extractor),
        threshold(_threshold)
    {
    }

    virtual ~TerraDecomClassifier()
    {
    }

    bool classify(const cv::Rect &roi, int &id)
    {
        cv::Mat descriptors;
        cv::Mat img_roi(image, roi);
        extractor->extract(img_roi, descriptors);
        float prob;
        classifier->predictClassProb(descriptors, id, prob);

        return prob < threshold;

    }

    void set_image(const cv::Mat &_image)
    {
        image = _image;
    }

private:
    RandomForest *classifier;
    Extractor    *extractor;
    float   threshold;
    cv::Mat image;
};
#endif // TERRA_DECOMPOSITION_CLASSIFIERS_HPP
