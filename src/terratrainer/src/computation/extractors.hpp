#ifndef EXTRACTORS_HPP
#define EXTRACTORS_HPP
#include <opencv2/nonfree/features2d.hpp>
#include "params.hpp"

namespace CMPExtractors {

inline cv::DescriptorExtractor* create(CMPParamsORB   &params)
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
    return ptr;
}

inline cv::DescriptorExtractor* create(CMPParamsSIFT  &params)
{
    cv::DescriptorExtractor* ptr  =
            new cv::SiftDescriptorExtractor(params.magnification,
                                            params.normalize,
                                            params.recalculateAngles);
    return ptr;
}
inline cv::DescriptorExtractor* create(CMPParamsSURF  &params)
{
    cv::DescriptorExtractor* ptr  =
            new cv::SurfDescriptorExtractor(params.octaves,
                                            params.octaveLayers,
                                            params.extended);
    return ptr;
}
inline cv::DescriptorExtractor* create(CMPParamsBRISK &params)
{
    cv::DescriptorExtractor* ptr =
            new cv::BRISK(params.radiusList,
                          params.numberList,
                          params.dMax,
                          params.dMin);
    return ptr;
}
inline cv::DescriptorExtractor* create(CMPParamsBRIEF &params)
{
    return new cv::BriefDescriptorExtractor(params.bytes);
}
inline cv::DescriptorExtractor* create(CMPParamsFREAK &params)
{
    cv::DescriptorExtractor *ptr =
            new cv::FREAK(params.orientationNormalized,
                          params.scaleNormalized,
                          params.patternScale,
                          params.octaves);
    return ptr;
}

inline void makeOpp(cv::DescriptorExtractor *ptr)
{
    ptr = new cv::OpponentColorDescriptorExtractor(ptr);
}
}


#endif // EXTRACTORS_HPP
