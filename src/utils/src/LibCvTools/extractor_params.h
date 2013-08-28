#ifndef EXTRACTOR_PARAMS_HPP
#define EXTRACTOR_PARAMS_HPP
#include <boost/shared_ptr.hpp>
/// SPARE DIRECT YAML INCLUSION
namespace YAML {
struct Emitter;
struct Node;
}

namespace cv_extraction {
struct KeypointParams {

    KeypointParams();

    float angle;
    float scale;
    int   octave;
    bool  soft_crop;
    bool  calc_angle;
    bool  dirty;
    void write(YAML::Emitter &emitter) const;
    void read(const YAML::Node &document);
};

struct ExtractorParams {
    typedef boost::shared_ptr<ExtractorParams> Ptr;

    enum Type   {NONE, ORB, BRISK, SIFT, SURF, BRIEF, FREAK, TSURF, LTP, LBP};
    ExtractorParams(const Type t = NONE, const int o = 1);
    ExtractorParams(const ExtractorParams &p);

    Type type;
    bool opp;
    bool color_extension;
    int  octaves;
    bool combine_descriptors;
    bool use_max_prob;
    bool dirty;

    virtual void write(YAML::Emitter &emitter) const;
    virtual bool read(const YAML::Node &document);
};

struct ParamsORB : public ExtractorParams
{

    ParamsORB();
    ParamsORB(const ParamsORB &p);

    double scale;
    int    WTA_K;
    int    patch_size;
    int    edge_threshold;

    void write(YAML::Emitter &emitter) const;
    bool read(const YAML::Node &document);
};

struct ParamsSURF : public ExtractorParams
{

    ParamsSURF();
    ParamsSURF(const ParamsSURF &p);

    int  octave_layers;
    bool extended;

    void write(YAML::Emitter &emitter) const;
    bool read(const YAML::Node &document);
};

struct ParamsSIFT : public ExtractorParams
{

    ParamsSIFT();
    ParamsSIFT(const ParamsSIFT &p);

    double magnification;
    bool   normalize;
    bool   recalculate_angles;

    void write(YAML::Emitter &emitter) const;
    bool read(const YAML::Node &document);
};


struct ParamsBRISK : public ExtractorParams
{

    ParamsBRISK();
    ParamsBRISK(const ParamsBRISK &p);

    int     thresh;
    float   scale;

    void write(YAML::Emitter &emitter) const;
    bool read(const YAML::Node &document);
};

struct ParamsBRIEF : public ExtractorParams
{

    ParamsBRIEF();
    ParamsBRIEF(const ParamsBRIEF &p);

    int bytes;  /// 16 32 64

    void write(YAML::Emitter &emitter) const;
    bool read(const YAML::Node &document);
};

struct ParamsFREAK : public ExtractorParams
{

    ParamsFREAK();
    ParamsFREAK(const ParamsFREAK &p);

    bool   orientation_normalized;
    bool   scale_normalized;
    double pattern_scale;

    void write(YAML::Emitter &emitter) const;
    bool read(const YAML::Node &document);
};

struct ParamsLTP : public ExtractorParams
{
    ParamsLTP();
    ParamsLTP(const ParamsLTP &p);

    double k;

    void write(YAML::Emitter &emitter) const;
    bool read(const YAML::Node &document);
};

struct ParamsLBP : public ExtractorParams
{

    ParamsLBP();
    ParamsLBP(const ParamsLBP &p);

    void write(YAML::Emitter &emitter) const;
    bool read(const YAML::Node &document);
};

struct ParamsTSURF : public ExtractorParams
{

    ParamsTSURF() :
        ExtractorParams(TSURF){}
};
}


#endif // EXTRACTOR_PARAMS_HPP
