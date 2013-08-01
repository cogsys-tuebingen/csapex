#include "cmp_core.h"
CMPCore::CMPCore()
{
}

bool CMPCore::load(const std::string image_path)
{
    raw_image_ = cv::imread(image_path);
    return !raw_image_.empty();
}

cv::Mat CMPCore::getImage() const
{
    return raw_image_.clone();
}

void CMPCore::addROI(const ROI &roi)
{

}

void CMPCore::makeOPP()
{
    extractor_ = new cv::OpponentColorDescriptorExtractor(extractor_);
}


void CMPCore::createORB  (CMPParamsORB &params)
{
    extractor_ = new cv::ORB(500,
                             params.scale,
                             params.levels,
                             31,
                             0,
                             params.WTA_K,
                             cv::ORB::HARRIS_SCORE,
                             params.patchSize);
    if(params.opp)
        makeOPP();
}

void CMPCore::createSIFT (CMPParamsSIFT &params)
{
    extractor_ = new cv::SiftDescriptorExtractor(params.magnification,
                                                 params.normalize,
                                                 params.recalculateAngles);
    if(params.opp)
        makeOPP();
}
void CMPCore::createSURF (CMPParamsSURF &params)
{
    extractor_= new cv::SurfDescriptorExtractor(params.octaves,
                                                params.octaveLayers,
                                                params.extended);
    if(params.opp)
        makeOPP();
}

void CMPCore::createBRISK(CMPParamsBRISK &params)
{
    extractor_ = new cv::BRISK(params.radiusList,
                               params.numberList,
                               params.dMax,
                               params.dMin);
    if(params.opp)
        makeOPP();
}

void CMPCore::createBRIEF(CMPParamsBRIEF &params)
{
    extractor_ = new cv::BriefDescriptorExtractor(params.bytes);
    if(params.opp)
        makeOPP();
}

void CMPCore::createFREAK(CMPParamsFREAK &params)
{
    extractor_= new cv::FREAK(params.orientationNormalized,
                              params.scaleNormalized,
                              params.patternScale,
                              params.octaves);
    if(params.opp)
        makeOPP();
}
