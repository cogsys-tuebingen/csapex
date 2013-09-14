#include "extractor_params.h"
#include <yaml-cpp/yaml.h>

using namespace cv_extraction;

KeypointParams::KeypointParams() :
    angle(0.f),
    scale(0.5f),
    octave(-1),
    soft_crop(true),
    calc_angle(false),
    dirty(true)
{
}

void KeypointParams::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "KEYPOINT" << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "angle"     << YAML::Value << angle;
    emitter << YAML::Key << "scale"     << YAML::Value << scale;
    emitter << YAML::Key << "octave"    << YAML::Value << octave;
    emitter << YAML::Key << "soft_crop" << YAML::Value << soft_crop;
    emitter << YAML::Key << "calc_angle"<< YAML::Value << calc_angle;
    emitter << YAML::EndMap;
}

void KeypointParams::read(const YAML::Node &document)
{
    try {

        const YAML::Node &data = document["KEYPOINT"];

        data["angle"]          >> angle;
        data["scale"]          >> scale;
        data["octave"]         >> octave;
        data["soft_crop"]      >> soft_crop;
        data["calc_angle"]     >> calc_angle;

    } catch (YAML::Exception e) {
        std::cerr << "ORB Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
    }
}

ExtractorParams::ExtractorParams(const Type t, const int o) :
    type(t),
    opp(false),
    color_extension(false),
    octaves(o),
    combine_descriptors(false),
    use_max_prob(false),
    dirty(true)
{
}

ExtractorParams::ExtractorParams(const ExtractorParams &p) :
    type(p.type),
    opp(p.opp),
    color_extension(p.color_extension),
    octaves(p.octaves),
    combine_descriptors(p.combine_descriptors),
    use_max_prob(p.use_max_prob),
    dirty(true)
{
}

void ExtractorParams::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "opp"       << YAML::Value << opp;
    emitter << YAML::Key << "color"     << YAML::Value << color_extension;
    emitter << YAML::Key << "octaves"   << YAML::Value << octaves;
    emitter << YAML::Key << "combine"   << YAML::Value << combine_descriptors;
    emitter << YAML::Key << "max_prob"  << YAML::Value << use_max_prob;
}

bool ExtractorParams::read(const YAML::Node &document)
{
    try {
        document["color"]       >> color_extension;
        document["opp"]         >> opp;
        document["octaves"]     >> octaves;
        document["combine"]     >> combine_descriptors;
        document["max_prob"]    >> use_max_prob;
        return true;
    } catch (YAML::Exception e) {
        std::cerr << "Extractor Parameters cannot read config : '" << e.what() << "'' !" << std::endl;
        return false;
    }
}

ParamsORB::ParamsORB() :
    ExtractorParams(ORB, 8),
    scale(1.2),
    WTA_K(2),
    patch_size(31),
    edge_threshold(0)
{
}

ParamsORB::ParamsORB(const ParamsORB &p) :
    ExtractorParams(p),
    scale(p.scale),
    WTA_K(p.WTA_K),
    patch_size(p.patch_size),
    edge_threshold(p.edge_threshold)
{
}


void ParamsORB::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "ORB" << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "scale"          << YAML::Value << scale;
    emitter << YAML::Key << "wta_k"          << YAML::Value << WTA_K;
    emitter << YAML::Key << "patch_size"     << YAML::Value << patch_size;
    emitter << YAML::Key << "edge_threshold" << YAML::Value << edge_threshold;
    ExtractorParams::write(emitter);
    emitter << YAML::EndMap;
}

bool ParamsORB::read(const YAML::Node &document)
{
    try {

        const YAML::Node &data = document["ORB"];
        data["scale"]          >> scale;
        data["wta_k"]          >> WTA_K;
        data["patch_size"]     >> patch_size;
        data["edge_threshold"] >> edge_threshold;
        return ExtractorParams::read(data);

    } catch (YAML::Exception e) {
        std::cerr << "ORB Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
        return false;
    }
}

ParamsSURF::ParamsSURF() :
    ExtractorParams(SURF, 4),
    threshold(100),
    octave_layers(3),
    extended(true)
{
}

ParamsSURF::ParamsSURF(const ParamsSURF &p) :
    ExtractorParams(p),
    octave_layers(p.octave_layers),
    extended(p.extended)
{
}

void ParamsSURF::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "SURF"           << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "threshold"      << YAML::Value << threshold;
    emitter << YAML::Key << "layers"         << YAML::Value << octave_layers;
    emitter << YAML::Key << "extended"       << YAML::Value << extended;
    ExtractorParams::write(emitter);
    emitter << YAML::EndMap;
}

bool ParamsSURF::read(const YAML::Node &document)
{
    try {
        const YAML::Node &data = document["SURF"];

        data["threshold"]       >> threshold;
        data["layers"]          >> octave_layers;
        data["extended"]        >> extended;
        return ExtractorParams::read(data);

    } catch (YAML::Exception e) {
        std::cerr << "SURF Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
        return false;
    }
}

ParamsSIFT::ParamsSIFT() :
    ExtractorParams(SIFT, 3),
    magnification(0.0),
    normalize(true),
    recalculate_angles(true)
{
}

ParamsSIFT::ParamsSIFT(const ParamsSIFT &p) :
    ExtractorParams(p),
    magnification(p.magnification),
    normalize(p.normalize),
    recalculate_angles(p.recalculate_angles)
{
}

void ParamsSIFT::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "SIFT"           << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "magnification"  << YAML::Value << magnification;
    emitter << YAML::Key << "normalize"      << YAML::Value << normalize;
    emitter << YAML::Key << "recalc_angles"  << YAML::Value << recalculate_angles;
    ExtractorParams::write(emitter);
    emitter << YAML::EndMap;
}

bool ParamsSIFT::read(const YAML::Node &document)
{
    try {

        const YAML::Node &data = document["SIFT"];

        data["magnification"]   >> magnification;
        data["normalize"]       >> normalize;
        data["recalc_angles"]   >> recalculate_angles;
        return  ExtractorParams::read(data);

    } catch (YAML::Exception e) {
        std::cerr << "SIFT Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
        return false;
    }
}

ParamsBRISK::ParamsBRISK() :
    ExtractorParams(BRISK, 3),
    thresh(30),
    scale(1.f)
{
}

ParamsBRISK::ParamsBRISK(const ParamsBRISK &p) :
    ExtractorParams(p),
    thresh(p.thresh),
    scale(p.scale)
{
}

void ParamsBRISK::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "BRISK"   << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "thresh"  << YAML::Value << thresh;
    emitter << YAML::Key << "scale"   << YAML::Value << scale;
    ExtractorParams::write(emitter);
    emitter << YAML::EndMap;
}

bool ParamsBRISK::read(const YAML::Node &document)
{
    try {
        const YAML::Node &data = document["BRISK"];

        data["thresh"]   >> thresh;
        data["scale"]    >> scale;
        return ExtractorParams::read(data);

    } catch (YAML::Exception e) {
        std::cerr << "BRISK Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
        return false;
    }
}

ParamsBRIEF::ParamsBRIEF() :
    ExtractorParams(BRIEF),
    bytes(16)
{
}

ParamsBRIEF::ParamsBRIEF(const ParamsBRIEF &p) :
    ExtractorParams(p),
    bytes(p.bytes)
{
}

void ParamsBRIEF::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "BRIEF"   << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "bytes"   << YAML::Value << bytes;
    ExtractorParams::write(emitter);
    emitter << YAML::EndMap;
}

bool ParamsBRIEF::read(const YAML::Node &document)
{
    try {

        const YAML::Node &data = document["BRIEF"];

        data["bytes"]   >> bytes;
        return ExtractorParams::read(data);

    } catch (YAML::Exception e) {
        std::cerr << "BRIEF Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
        return false;
    }

}

ParamsFREAK::ParamsFREAK() :
    ExtractorParams(FREAK, 4),
    orientation_normalized(true),
    scale_normalized(true),
    pattern_scale(22.0)
{
}

ParamsFREAK::ParamsFREAK(const ParamsFREAK &p) :
    ExtractorParams(p),
    orientation_normalized(p.orientation_normalized),
    scale_normalized(p.scale_normalized),
    pattern_scale(p.pattern_scale)
{
}

void ParamsFREAK::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "FREAK"   << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "orientation_norm" << YAML::Value << orientation_normalized;
    emitter << YAML::Key << "scale_normalized" << YAML::Value << scale_normalized;
    emitter << YAML::Key << "pattern_scale"    << YAML::Value << pattern_scale;
    ExtractorParams::write(emitter);
    emitter << YAML::EndMap;
}

bool ParamsFREAK::read(const YAML::Node &document)
{
    try {

        const YAML::Node &data = document["FREAK"];

        data["orientation_norm"] >> orientation_normalized;
        data["scale_normalized"] >> scale_normalized;
        data["pattern_scale"]    >> pattern_scale;
        return ExtractorParams::read(data);

    } catch (YAML::Exception e) {
        std::cerr << "FREAK Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
        return false;
    }

}

ParamsLTP::ParamsLTP() :
    ExtractorParams(LTP),
    k(0.0)
{
    combine_descriptors = true;
}

ParamsLTP::ParamsLTP(const ParamsLTP &p) :
    ExtractorParams(p),
    k(p.k)
{
    combine_descriptors = p.combine_descriptors;
}


void ParamsLTP::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "LTP" << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "k" << YAML::Value << k;
    ExtractorParams::write(emitter);
    emitter << YAML::EndMap;
}

bool ParamsLTP::read(const YAML::Node &document)
{
    try {
        const YAML::Node &data = document["LTP"];
        data["k"] >> k;
        return ExtractorParams::read(data);
    } catch (YAML::Exception e) {
        std::cerr << "LTP Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
        return false;
    }
}


ParamsLBP::ParamsLBP() :
    ExtractorParams(LBP)
{
}

ParamsLBP::ParamsLBP(const ParamsLBP &p) :
    ExtractorParams(p)
{
}

void ParamsLBP::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "LBP" << YAML::Value;
    emitter << YAML::BeginMap;
    ExtractorParams::write(emitter);
    emitter << YAML::EndMap;
}

bool ParamsLBP::read(const YAML::Node &document)
{
    try {
        const YAML::Node &data = document["LBP"];
        return ExtractorParams::read(data);
    } catch (YAML::Exception e) {
        std::cerr << "LBP Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
        return false;
    }
}
