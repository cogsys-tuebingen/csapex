/// HEADER
#include "ros_config.h"

Config RosConfig::importFromNodeHandle(ros::NodeHandle& nh)
{

    RosConfig result;

    nh.param("bin_count", result.bin_count, 16);
    nh.param("db_file", result.db_file, result.db_file);

    return result;
}

#define COPY_PARAM(target, def, var) \
    target->var = def.var

namespace
{
void copy(Config* target, const robot_detection::GlobalConfig& def)
{
    COPY_PARAM(target, def, keypoint_name);
    COPY_PARAM(target, def, descriptor_name);
    COPY_PARAM(target, def, angle_offset);
    COPY_PARAM(target, def, sample_threshold);
    COPY_PARAM(target, def, score_threshold);
    COPY_PARAM(target, def, octaves);
    COPY_PARAM(target, def, extractor_threshold);
    COPY_PARAM(target, def, matcher_threshold);
    COPY_PARAM(target, def, min_points);
    COPY_PARAM(target, def, use_pruning);
    COPY_PARAM(target, def, crop_test);
    COPY_PARAM(target, def, interactive);
}
}

Config RosConfig::import(robot_detection::GlobalConfig& cfg, int level)
{
    Config latest = RosConfig::getGlobal();

    if(level != -1) {
        // not initial!
        copy(&latest, cfg);
    }

    latest.replaceGlobal();

    return latest;
}

void RosConfig::make_defaults()
{
    robot_detection::GlobalConfig def = robot_detection::GlobalConfig::__getDefault__();

    copy(this, def);
}

#undef COPY_PARAM
