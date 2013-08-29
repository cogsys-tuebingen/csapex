#include "terra_decomposition_classifier.h"

TerraDecomClassifier::TerraDecomClassifier(const float _threshold) :
    threshold(_threshold),
    use_max_prob(false)
{
}

TerraDecomClassifier::~TerraDecomClassifier()
{
}

void TerraDecomClassifier::setClassifier(RandomForest::Ptr _classifier)
{
    classifier = _classifier;
}

void TerraDecomClassifier::setExtractor(cv_extraction::Extractor::Ptr _extractor)
{
    extractor  = _extractor;
}

void TerraDecomClassifier::setUseMaxProb(bool value)
{
    use_max_prob = value;
}

bool TerraDecomClassifier::classify(const Rect &roi)
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
        if(use_max_prob)
            classifier->predictClassProbMultiSampleMax(descriptors, last_id, last_prob);
        else
            classifier->predictClassProbMultiSample(descriptors, last_id, last_prob);
    } else {
        classifier->predictClassProb(descriptors, last_id, last_prob);
    }

    return last_prob < threshold;

}

float TerraDecomClassifier::get_prob()
{
    return last_prob;
}

int TerraDecomClassifier::get_id()
{
    return last_id;
}

void TerraDecomClassifier::set_image(const Mat &_image)
{
    image = _image;
}
