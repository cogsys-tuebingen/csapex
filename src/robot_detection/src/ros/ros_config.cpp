/// HEADER
#include "ros_config.h"

Config RosConfig::importFromNodeHandle(ros::NodeHandle& nh)
{

    RosConfig result;

    int bin_count;
    nh.param("bin_count", bin_count, 16);
    result["bin_count"] = bin_count;

    std::string db_file;
    nh.param("db_file", db_file, db_file);
    result["db_file"] = db_file;

    return result;
}

#define COPY_PARAM(target, def, var) \
    target->var = def.var

#define COPY_PARAM_MAP(target, def, var) \
    (*target)["var"] = def.var

namespace
{
void copy(Config* target, const robot_detection::GlobalConfig& def)
{
    COPY_PARAM_MAP(target, def, keypoint_name);
    COPY_PARAM_MAP(target, def, descriptor_name);
    COPY_PARAM_MAP(target, def, angle_offset);
    COPY_PARAM_MAP(target, def, sample_threshold);
    COPY_PARAM_MAP(target, def, score_threshold);
    COPY_PARAM_MAP(target, def, matcher_threshold);
    COPY_PARAM_MAP(target, def, octaves);
    COPY_PARAM_MAP(target, def, extractor_threshold);
    COPY_PARAM_MAP(target, def, min_points);
    COPY_PARAM_MAP(target, def, use_pruning);
    COPY_PARAM_MAP(target, def, crop_test);
    COPY_PARAM_MAP(target, def, interactive);
}
}

Config RosConfig::import(robot_detection::GlobalConfig& cfg, int level)
{
    Config latest = RosConfig::instance();

    if(level != -1) {
        // not initial!
        copy(&latest, cfg);
    }

    latest.replaceInstance();

    return latest;
}

void RosConfig::make_defaults()
{
    robot_detection::GlobalConfig def = robot_detection::GlobalConfig::__getDefault__();

    copy(this, def);
}

#undef COPY_PARAM
