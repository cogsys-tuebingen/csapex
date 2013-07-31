/// HEADER
#include "config.h"

/// SYSTEM
#include <sstream>


Config Config::latest;

boost::signals2::signal<void(const Config&)> Config::replace;

Config::Config()
{
    make_defaults();
    init();
}

void Config::make_defaults()
{
    /// defaults
    keypoint_name = "ORB";
    descriptor_name = "ORB";
    angle_offset = 0.0;
    sample_threshold = 0.0;
    score_threshold = 0.7;
    octaves = 8;
    extractor_threshold = 50;
    matcher_threshold = 0.95;
    min_points = 3;
    use_pruning = 1;
    crop_test = 0;
    interactive = 1;
}

void Config::init()
{
    setDescriptorType(descriptor_name);
    setKeypointType(keypoint_name);

    config_dir = std::string(getenv("RABOT")) + "/Config/RobotDetection/";
    result_dir = config_dir + "results/";
    batch_dir = config_dir + "feature_data_training/";
    ref_dir = config_dir + "reference/";
    db_file = config_dir + "robot.db";
    db_imgs = config_dir + "robot_imgs/";

    db_type = Types::Strategy::BIN;
    //db_type = Types::Strategy::NAIVE;

    bin_count = 16;
    bin_max_poses_count = 10;
    bin_min_feature_count = 20;
    gui_enabled = true;

    name = "unnamed";
}

void Config::replaceGlobal() const
{
    latest = *this;
    replace(*this);
}

Config Config::getGlobal()
{
    return latest;
}

const std::string& Config::getKeypointType() const
{
    return keypoint_name;
}

const std::string& Config::getDescriptorType() const
{
    return descriptor_name;
}

void Config::setKeypointType(const std::string& type)
{
    std::string low = type;
    std::transform(low.begin(), low.end(), low.begin(), ::tolower);
    keypoint_name = low;
}

void Config::setDescriptorType(const std::string& type)
{
    std::string low = type;
    std::transform(low.begin(), low.end(), low.begin(), ::tolower);
    descriptor_name = low;
}

std::string Config::getDescription() const
{
    std::stringstream name;
    name << keypoint_name << "_"
         << descriptor_name << "_"
         << bin_count << "_" << extractor_threshold
         << "_" << min_points << "_" << octaves
         << "_" << matcher_threshold
         << "_" << (use_pruning ? "prune" : "noprune") << "_" << (crop_test ? "crop" : "nocrop");
    return name.str();
}

std::string Config::computeHash() const
{
    return getDescription();
}
