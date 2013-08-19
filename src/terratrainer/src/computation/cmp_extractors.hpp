#ifndef EXTRACTORS_HPP
#define EXTRACTORS_HPP
#include <opencv2/nonfree/features2d.hpp>
#include "params.hpp"

namespace CMPExtractors {
inline void makeOpp(cv::DescriptorExtractor *ptr)
{
    ptr = new cv::OpponentColorDescriptorExtractor(ptr);
}

inline cv::DescriptorExtractor* prepare(CMPParamsORB   &params)
{
    cv::DescriptorExtractor* ptr  =
            new cv::ORB(500,
                        params.scale,
                        params.levels,
                        31,
                        0,
                        params.WTA_K,
                        cv::ORB::HARRIS_SCORE,
                        params.patchSize);

    if(params.opp)
        CMPExtractors::makeOpp(ptr);

    return ptr;
}

inline cv::DescriptorExtractor* prepare(CMPParamsSIFT  &params)
{
    cv::DescriptorExtractor* ptr  =
            new cv::SiftDescriptorExtractor(params.magnification,
                                            params.normalize,
                                            params.recalculateAngles);
    if(params.opp)
        CMPExtractors::makeOpp(ptr);

    return ptr;
}
inline cv::DescriptorExtractor* prepare(CMPParamsSURF  &params)
{
    cv::DescriptorExtractor* ptr  =
            new cv::SurfDescriptorExtractor(params.octaves,
                                            params.octaveLayers,
                                            params.extended);
    if(params.opp)
        CMPExtractors::makeOpp(ptr);
    return ptr;
}
inline cv::DescriptorExtractor* prepare(CMPParamsBRISK &params)
{
    cv::DescriptorExtractor* ptr =
            new cv::BRISK(params.radiusList,
                          params.numberList,
                          params.dMax,
                          params.dMin);
    if(params.opp)
        CMPExtractors::makeOpp(ptr);
    return ptr;
}
inline cv::DescriptorExtractor* prepare(CMPParamsBRIEF &params)
{
    return new cv::BriefDescriptorExtractor(params.bytes);
}
inline cv::DescriptorExtractor* prepare(CMPParamsFREAK &params)
{
    cv::DescriptorExtractor *ptr =
            new cv::FREAK(params.orientationNormalized,
                          params.scaleNormalized,
                          params.patternScale,
                          params.octaves);
    if(params.opp)
        CMPExtractors::makeOpp(ptr);

    return ptr;
}
}


#endif // EXTRACTORS_HPP
