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
    int    cell_height;
    int    cell_width;
};

struct CMPQuadParams {
    int     min_height;
    int     min_width;
    float   min_prob;
};

struct CMPKeypointParams {
    CMPKeypointParams() : angle(0.f), size(5.f){}

    float angle;
    float size;
};

struct CMPExtractorParams {
    enum Type   {ORB, BRISK, SIFT, SURF, BRIEF, FREAK, TSURF, LTP};
    CMPExtractorParams(const Type t) :
        type(t),
        opp(false){}

    Type type;
    bool opp;
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
        CMPExtractorParams(SURF),
        octaves(4),
        octaveLayers(3),
        extended(true){}

    int  octaves;
    int  octaveLayers;
    bool extended;
};

struct CMPParamsSIFT : public CMPExtractorParams
{
    CMPParamsSIFT() :
        CMPExtractorParams(SIFT),
        magnification(0.0),
        octaves(3),
        normalize(true),
        recalculateAngles(true){}

    double magnification;
    int    octaves;
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
        CMPExtractorParams(FREAK),
        orientationNormalized(true),
        scaleNormalized(true),
        patternScale(22.0),
        octaves(4){}

    bool   orientationNormalized;
    bool   scaleNormalized;
    double patternScale;
    int    octaves;
};

struct CMPParamsLTP : public CMPExtractorParams
{
    CMPParamsLTP() :
        CMPExtractorParams(LTP){}
};

struct CMPParamsTSURF : public CMPExtractorParams
{
    CMPParamsTSURF() :
        CMPExtractorParams(TSURF){}
};


#endif // PARAMS_HPP
