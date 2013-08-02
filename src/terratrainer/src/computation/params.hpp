#ifndef PARAMS_HPP
#define PARAMS_HPP
/// SYSTEM
#include <vector>

struct CMPParams {
    enum Type   {ORB, BRISK, SIFT, SURF, BRIEF, FREAK};
    CMPParams(const Type t) :
        type(t),
        opp(false){}

    Type type;
    bool opp;
};

struct CMPParamsORB : public CMPParams
{
    CMPParamsORB() :
        CMPParams(ORB),
        levels(8),
        scale(1.2),
        WTA_K(2),
        patchSize(31){}

    int    levels;
    double scale;
    int    WTA_K;
    int    patchSize;
};

struct CMPParamsSURF : public CMPParams
{
    CMPParamsSURF() :
        CMPParams(SURF),
        octaves(4),
        octaveLayers(3),
        extended(true){}

    int  octaves;
    int  octaveLayers;
    bool extended;
};

struct CMPParamsSIFT : public CMPParams
{
    CMPParamsSIFT() :
        CMPParams(SIFT),
        magnification(0.0),
        octaves(3),
        normalize(true),
        recalculateAngles(true){}

    double magnification;
    int    octaves;
    bool   normalize;
    bool   recalculateAngles;
};


struct CMPParamsBRISK : public CMPParams
{
    CMPParamsBRISK() :
        CMPParams(BRISK),
        dMax(5.85),
        dMin(8.2){}

    std::vector<float>   radiusList;
    std::vector<int>     numberList;
    double               dMax;
    double               dMin;

};

struct CMPParamsBRIEF : public CMPParams
{
    CMPParamsBRIEF() :
        CMPParams(BRIEF),
        bytes(16){}

    int bytes;  /// 16 32 64
};

struct CMPParamsFREAK : public CMPParams
{
    CMPParamsFREAK() :
        CMPParams(FREAK),
        orientationNormalized(true),
        scaleNormalized(true),
        patternScale(22.0),
        octaves(4){}

    bool   orientationNormalized;
    bool   scaleNormalized;
    double patternScale;
    int    octaves;
};



#endif // PARAMS_HPP
