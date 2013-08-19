#ifndef PARAMS_HPP
#define PARAMS_HPP
/// SYSTEM
#include <vector>

struct CMPForestParams {
    /**
     *CV_TERMCRIT_ITER = 1;
     *CV_TERMCRIT_EPS  = 2;
     */

    CMPForestParams() :
        max_depth(25), min_samples(5), regression(0),
        surrogates(false), max_categories(15),
        priors(0), variable_importance(false),
        nactive_variables(4), max_trees(100),
        accurracy(0.01f),
        termination_criteria(1 | 2){}

    int                max_depth;
    int                min_samples;
    float              regression;
    bool               surrogates;
    int                max_categories;
    std::vector<float> priors;
    bool               variable_importance;
    int                nactive_variables;
    int                max_trees;
    float              accurracy;
    int                termination_criteria;
};

struct CMPGridParams {
    CMPGridParams() : cell_height(10), cell_width(10), height(48), width(64){}
    int    cell_height;
    int    cell_width;
    int    height;
    int    width;
};

struct CMPQuadParams {
    CMPQuadParams() : min_height(10), min_width(10), min_prob(0.75){}
    int     min_height;
    int     min_width;
    float   min_prob;
};

struct CMPKeypointParams {
    CMPKeypointParams() : angle(0.f), scale(0.5f), octave(-1), soft_crop(true){}

    float angle;
    float scale;
    int   octave;
    bool  soft_crop;
};

struct CMPExtractorParams {
    enum Type   {ORB, BRISK, SIFT, SURF, BRIEF, FREAK, TSURF, LTP, LBP};
    CMPExtractorParams(const Type t, const int o = 0) :
        type(t),
        opp(false),
        colorExtension(false),
        octaves(o),
        combine_descriptors(false){}

    Type type;
    bool opp;
    bool colorExtension;
    int  octaves;
    bool combine_descriptors;

};

struct CMPParamsORB : public CMPExtractorParams
{
    CMPParamsORB() :
        CMPExtractorParams(ORB),
        levels(8),
        scale(1.2),
        WTA_K(2),
        patchSize(31){}

    int    levels;
    double scale;
    int    WTA_K;
    int    patchSize;
};

struct CMPParamsSURF : public CMPExtractorParams
{
    CMPParamsSURF() :
        CMPExtractorParams(SURF, 4),
        octaveLayers(3),
        extended(true){}

    int  octaveLayers;
    bool extended;
};

struct CMPParamsSIFT : public CMPExtractorParams
{
    CMPParamsSIFT() :
        CMPExtractorParams(SIFT, 3),
        magnification(0.0),
        normalize(true),
        recalculateAngles(true){}

    double magnification;
    bool   normalize;
    bool   recalculateAngles;
};


struct CMPParamsBRISK : public CMPExtractorParams
{
    CMPParamsBRISK() :
        CMPExtractorParams(BRISK),
        dMax(5.85),
        dMin(8.2){}

    std::vector<float>   radiusList;
    std::vector<int>     numberList;
    double               dMax;
    double               dMin;

};

struct CMPParamsBRIEF : public CMPExtractorParams
{
    CMPParamsBRIEF() :
        CMPExtractorParams(BRIEF),
        bytes(16){}

    int bytes;  /// 16 32 64
};

struct CMPParamsFREAK : public CMPExtractorParams
{
    CMPParamsFREAK() :
        CMPExtractorParams(FREAK, 4),
        orientationNormalized(true),
        scaleNormalized(true),
        patternScale(22.0){}

    bool   orientationNormalized;
    bool   scaleNormalized;
    double patternScale;
};

struct CMPParamsLTP : public CMPExtractorParams
{
    CMPParamsLTP() :
        CMPExtractorParams(LTP),
        k(0.0)
    {
        combine_descriptors = true;
    }

    double k;
};

struct CMPParamsLBP : public CMPExtractorParams
{
    CMPParamsLBP() :
        CMPExtractorParams(LBP){}
};

struct CMPParamsTSURF : public CMPExtractorParams
{
    CMPParamsTSURF() :
        CMPExtractorParams(TSURF){}
};


#endif // PARAMS_HPP
