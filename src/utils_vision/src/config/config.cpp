/// HEADER
#include "config.h"

/// SYSTEM
#include <sstream>

using namespace param;

boost::signals2::signal<void(const Config&)> Config::replace;

Config::Config()
{
    make_defaults();
    init();
}

void Config::make_defaults()
{
    /// defaults
    parameters["keypointType"] = "ORB";
    parameters["descriptorType"] = "ORB";
    parameters["angle_offset"] = 0.0;
    parameters["sample_threshold"] = 0.0;
    parameters["score_threshold"] = 0.7;
    parameters["matcher_threshold"] = 0.95;
    parameters["octaves"] = 8;
    parameters["extractor_threshold"] = 50;
    parameters["min_points"] = 3;
    parameters["use_pruning"] = true;
    parameters["crop_test"] = false;
    parameters["interactive"] = true;
}

void Config::init()
{
    setDescriptorType(parameters["keypointType"]);
    setKeypointType(parameters["descriptorType"]);

    std::string rabot;
    if(getenv("RABOT")) {
        rabot = getenv("RABOT");
    } else {
        WARN("RABOT environment variable not found!!!!");
    }


    parameters["config_dir"] = rabot + "/Config/RobotDetection/";
    parameters["result_dir"] = parameters["config_dir"].as<std::string>() + "results/";
    parameters["batch_dir"] = parameters["config_dir"].as<std::string>() + "feature_data_training/";
    parameters["ref_dir"] = parameters["config_dir"].as<std::string>() + "reference/";
    parameters["db_file"] = parameters["config_dir"].as<std::string>() + "robot.db";
    parameters["db_imgs"] = parameters["config_dir"].as<std::string>() + "robot_imgs/";

    parameters["db_type"] = Types::Strategy::BIN;
    //db_type = Types::Strategy::NAIVE;

    parameters["bin_count"] = 16;
    parameters["bin_max_poses_count"] = 10;
    parameters["bin_min_feature_count"] = 20;
    parameters["gui_enabled"] = true;

    parameters["name"] = "unnamed";
}

void Config::replaceInstance() const
{
    latest_config().value = *this;
    replace(*this);
}

impl::ConfigHolder& Config::latest_config()
{
    static impl::ConfigHolder instance;
    return instance;
}

Config Config::instance()
{
    return latest_config().value;
}

void Config::setKeypointType(const std::string& type)
{
    std::string low = type;
    std::transform(low.begin(), low.end(), low.begin(), ::tolower);
    parameters["keypointName"] = low;
}

void Config::setDescriptorType(const std::string& type)
{
    std::string low = type;
    std::transform(low.begin(), low.end(), low.begin(), ::tolower);
    parameters["descriptorName"] = low;
}

std::string Config::getDescription() const
{
    std::stringstream name;
    name << parameters.at("keypointName").as<std::string>() << "_"
         << parameters.at("descriptorName").as<std::string>() << "_"
         << parameters.at("bin_count").as<int>() << "_"
         << parameters.at("extractor_threshold").as<int>() << "_"
         << parameters.at("min_points").as<int>() << "_"
         << parameters.at("octaves").as<int>() << "_"
         << parameters.at("matcher_threshold").as<double>() << "_"
         << (parameters.at("use_pruning").as<bool>() ? "prune" : "noprune") << "_"
         << (parameters.at("crop_test").as<bool>() ? "crop" : "nocrop");
    return name.str();
}

std::string Config::computeHash() const
{
    return getDescription();
}

Parameter& Config::getParameter(const std::string &name)
{
    try {
        return parameters.at(name);
    } catch(const std::out_of_range& e) {
        throw std::logic_error(std::string("unknown parameter '") + name + ")");
    }
}

const Parameter& Config::getConstParameter(const std::string &name) const
{
    try {
        return parameters.at(name);
    } catch(const std::out_of_range& e) {
        throw std::logic_error(std::string("unknown parameter '") + name + ")");
    }
}
