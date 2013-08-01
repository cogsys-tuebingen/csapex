#ifndef PARAMS_HPP
#define PARAMS_HPP
/// SYSTEM
#include <vector>

struct CMPParams {
    enum Type   {ORB, BRISK, SIFT, SURF, BRIEF, FREAK};
    CMPParams(const Type t) : type(t){}
    Type type;
    bool opp;
};

struct CMPParamsORB : public CMPParams
{
    CMPParamsORB() : CMPParams(ORB){}
    int    levels;
    double scale;
    int    WTA_K;
    int    patchSize;
};

struct CMPParamsSURF : public CMPParams
{
    CMPParamsSURF() : CMPParams(SURF){}
    int  octaves;
    int  octaveLayers;
    bool extended;
};

struct CMPParamsSIFT : public CMPParams
{
    CMPParamsSIFT() : CMPParams(SIFT){}
    double magnification;
    int    octaves;
    bool   normalize;
    bool   recalculateAngles;
};


struct CMPParamsBRISK : public CMPParams
{
    CMPParamsBRISK() : CMPParams(BRISK){}
    std::vector<float>   radiusList;
    std::vector<int>     numberList;
    double               dMax;
    double               dMin;

};

struct CMPParamsBRIEF : public CMPParams
{
    CMPParamsBRIEF() : CMPParams(BRIEF){}
    int bytes;  /// 16 32 64
};

struct CMPParamsFREAK : public CMPParams
{
    CMPParamsFREAK() : CMPParams(FREAK){}
    bool   orientationNormalized;
    bool   scaleNormalized;
    double patternScale;
    int    octaves;
};



#endif // PARAMS_HPP
