#ifndef TERRA_DECOMPOSITION_CLASSIFIERS_H
#define TERRA_DECOMPOSITION_CLASSIFIERS_H
#include "decomposition_classifiers.hpp"

class TerraDecomClassifier : public DecompositionClassifier
{
    ///
public :
    typedef boost::shared_ptr<TerraDecomClassifier> Ptr;

    TerraDecomClassifier(const float _threshold);
    virtual ~TerraDecomClassifier();

    void  setClassifier      (RandomForest::Ptr _classifier);
    void  setExtractor       (cv_extraction::Extractor::Ptr _extractor);
    void  setUseMaxProb      (bool value);
    void  setUseColorExt     (bool value);
    void  classify           (const std::vector<cv::Rect> rois, std::vector<bool> classification);
    bool  classify           (const cv::Rect &roi);
    float get_prob           ();
    int   get_id             ();
    void  set_image          (const cv::Mat &_image);


private:
    RandomForest::Ptr                    classifier;
    cv_extraction::Extractor::Ptr        extractor;
    float                                threshold;
    bool                                 use_max_prob;
    bool                                 color_ext;
    cv::Mat                              image;
    float                                last_prob;
    int                                  last_id;
};
#endif // TERRA_DECOMPOSITION_CLASSIFIERS_H
