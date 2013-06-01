/**
 * @file OccupancyGridWrapper.cpp
 * @date Jan 2012
 * @author marks
 */

// project
#include "OccupancyGridWrapper.h"

using namespace lib_ros_util;

OccupancyGridWrapper::OccupancyGridWrapper( nav_msgs::OccupancyGrid* map,
                                            uint8_t lo_thres, uint8_t up_thres )
    : map_( map ),
      lower_thres_( lo_thres ),
      upper_thres_( up_thres )
{ /* Nothing left */ }
