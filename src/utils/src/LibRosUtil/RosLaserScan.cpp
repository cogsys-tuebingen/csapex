/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Marks
 @date 2011

 @file RosLaserScan.cpp
*/

// C/C++
#include <iostream>
#include <cmath>

// Project
#include "RosLaserScan.h"

using namespace std;

namespace lib_ros_util {

RosLaserScan::RosLaserScan()
    : angle_min_( 0 ), angle_max_( 0 ), range_min_( 0 ), range_max_( 0 ), scan_obtained_( false )
{}

bool RosLaserScan::update( const sensor_msgs::LaserScan::ConstPtr &scan ) {
    // Check number of beams
    if ( scan->ranges.size() <= 0 ) {
        return false; // This scan is useless
    }

    // Check min/max angle etc
    bool params_changed = checkSettings( scan );

    // Write info if this is the first scan
    if ( !scan_obtained_ ) {
        ROS_DEBUG( "[RosLaserScan] Obtained first laser scan. Parameters are:" );
        ROS_DEBUG( "[RosLaserScan] Min angle: %f Max angle: %f", angle_min_, angle_max_ );
        ROS_DEBUG( "[RosLaserScan] Min range: %f Max range: %f", range_min_, range_max_ );
        ROS_DEBUG( "[RosLaserScan] Number of beams: %d", (int)scan->ranges.size());
        ROS_DEBUG( "[RosLaserScan] Frame id: %s", scan->header.frame_id.c_str());
    }

    // Write info if parameters changed
    if ( params_changed && scan_obtained_ ) {
        ROS_INFO( "[RosLaserScan] Laser scan parameters changed. Recomputing beam settings!" );
    }

    // Validate and set range readings, compute obstacle positions
    scan_obtained_ = true;
    float range;
    lib_laser_processing::LaserBeam* b;
    for ( size_t i = 0; i < beams_.size(); ++i ) {
        b = &beams_[i];
        range = scan->ranges[i];
        if ( range >= range_min_ && range <= range_max_ ) {
            b->range = range;
            b->pos(0) = range * cos_[i];
            b->pos(1) = range * sin_[i];
            b->valid = true;
        } else {
            b->valid = false;
        }
    }

    return true;
}

bool RosLaserScan::checkSettings( const sensor_msgs::LaserScan::ConstPtr &scan ) {
    // Min/max angles, min/max range or number of beams changed?
    if ( angle_min_ != scan->angle_min
         || angle_max_ != scan->angle_max
         || beams_.size() != scan->ranges.size()
         || range_min_ != scan->range_min
         || range_max_ != scan->range_max ) {

        // Store min/max range, values will be checked each time to validate a range reading
        range_min_ = scan->range_min;
        range_max_ = scan->range_max;

        // Compute beam settings since we have to know the yaw angle etc. for each beam
        angle_min_ = scan->angle_min;
        angle_max_ = scan->angle_max;
        beams_.resize( scan->ranges.size());
        sin_.resize( scan->ranges.size());
        cos_.resize( scan->ranges.size());
        for ( size_t i = 0; i < beams_.size(); ++i ) {
            beams_[i].yaw = (float)(i) * scan->angle_increment + angle_min_;
            sin_[i] = sin( beams_[i].yaw );
            cos_[i] = cos( beams_[i].yaw );
            beams_[i].valid = false;
        }

        return true; // Something changed
    }

    return false; // Nothing changed
}

} // namespace
