#include "extractor.h"
#include "feature_extractor.h"
#include "pattern_extractor.h"
#include "yaml-cpp/yaml.h"

using namespace cv_extraction;

ExtractorParams Extractor::params()
{
    return *ext_params_;
}

cv::Scalar Extractor::extractMeanColorRGBYUV(const cv::Mat &img)
{
    cv::Mat convert;
    cv::cvtColor(img, convert, CV_BGR2YUV);

    return cv::mean(convert);
}

void Extractor::addColorExtension(cv::Mat &descriptor, const cv::Scalar &color)
{
    cv::Mat tmp(descriptor.rows, descriptor.cols + 2, descriptor.type(),cv::Scalar::all(0));
    cv::Mat tmp_roi(tmp, cv::Rect(0,0, descriptor.cols, descriptor.rows));
    tmp.col(tmp.cols - 2).setTo(color[1]);
    tmp.col(tmp.cols - 1).setTo(color[2]);
    descriptor.copyTo(tmp_roi);
    tmp.copyTo(descriptor);
}

void Extractor::read(const YAML::Node &document, Extractor::Ptr &extractor,
                     ExtractorParams::Ptr &params, KeypointParams &key)
{
    cv_extraction::FeatureExtractor::Ptr feat(new cv_extraction::FeatureExtractor);
    cv_extraction::PatternExtractor::Ptr patt(new cv_extraction::PatternExtractor);
    try {
        cv_extraction::ParamsORB  orb;
        if(orb.read(document)) {
            feat->setParams(orb);
            extractor = feat;
            params.reset(new cv_extraction::ParamsORB(orb));
        }
        cv_extraction::ParamsSURF surf;
        if(surf.read(document)) {
            feat->setParams(surf);
            extractor = feat;
            params.reset(new cv_extraction::ParamsSURF(surf));
        }
        cv_extraction::ParamsSIFT sift;
        if(sift.read(document)) {
            feat->setParams(sift);
            extractor = feat;
            params.reset(new cv_extraction::ParamsSIFT(sift));
        }
        cv_extraction::ParamsBRISK brisk;
        if(brisk.read(document)) {
            feat->setParams(brisk);
            extractor = feat;
            params.reset(new cv_extraction::ParamsBRISK(brisk));
        }
        cv_extraction::ParamsBRIEF brief;
        if(brief.read(document)) {
            feat->setParams(brief);
            extractor = feat;
            params.reset(new cv_extraction::ParamsBRIEF(brief));
        }
        cv_extraction::ParamsFREAK freak;
        if(freak.read(document)) {
            feat->setParams(freak);
            extractor = feat;
            params.reset(new cv_extraction::ParamsFREAK(freak));
        }
        cv_extraction::ParamsLBP lbp;
        if(lbp.read(document)) {
            patt->setParams(lbp);
            extractor = patt;
            params.reset(new cv_extraction::ParamsLBP(lbp));
        }
        cv_extraction::ParamsLTP ltp;
        if(ltp.read(document)) {
            patt->setParams(ltp);
            extractor = patt;
            params.reset(new cv_extraction::ParamsLTP(ltp));
        }

        /// KEYPOINT PARAMS
        key.read(document);
        feat->setKeyPointParams(key);
        std::cout << "Loaded extractor!" << std::endl;
    } catch (YAML::Exception e) {
        std::cerr << "Problems reading preferences : '" << e.what() <<"' !" << std::endl;
    }
}
