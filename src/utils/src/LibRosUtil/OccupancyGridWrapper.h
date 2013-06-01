/**
 * @file OccupancyGridWrapper.h
 * @date Jan 2012
 * @author marks
 */

#ifndef OCCUPANCYGRIDWRAPPER_H
#define OCCUPANCYGRIDWRAPPER_H

// ROS
#include <nav_msgs/OccupancyGrid.h>

// LibPath
#include <utils/LibPath/common/GridMap2d.h>

namespace lib_ros_util {

/**
 * @class OccupancyGridWrapper
 * @brief A wrapper object for ROS OccupancyGrid maps.
 *
 * This class makes it possible to use ROS maps as input for
 * the algorithms in utils/LibPath.
 *
 * @attention The ROS OccupancyGrid map uses signed 8-bit integers
 * to represent the cell values. The value -1 represents "no information".
 * The value 0 represents a surely open cell and 100 represents a lethal obstacle.
 * So the data range is -1 to 100. We are using unsigned 8-bit integers and
 * cast the cell values. So -1 becomes 255 and every cell value > 100 represents
 * "no information"!
 */
class OccupancyGridWrapper : public lib_path::GridMap2d
{
public:

    /**
     * @brief Create the object.
     *
     * @attention There is no internal copy of the map. The given pointer
     * should be valid as long as this wrapper is in use.
     *
     * @param map The underlying map. The data range of the cell values
     *      should be -1 to 100.
     * @param lo_thres Lower threshold. Every cell with a value <= this threshold
     *      is free.
     * @param up_thres Upper threshold. Every cell with a value >= this threshold
     *      represents a lethal obstacle.
     */
    OccupancyGridWrapper( nav_msgs::OccupancyGrid* map, uint8_t lo_thres = 20, uint8_t up_thres = 80 );

    virtual ~OccupancyGridWrapper() { /* Nothing to do */ }

    /**
     * @brief Set the underlying map.
     *
     * @attention There is no internal copy of the map. The given pointer
     * should be valid as long as this wrapper is in use.
     *
     * @param map The underlying map. The data range of the cell values
     *      should be -1 to 100.
     */
    void setMap( nav_msgs::OccupancyGrid* map )
        { map_ = map; }

    /**
     * @brief Set the lower threshold.
     * Every cell with a value <= this threshold is free.
     * @param lo_thres The new lower threshold from 0 to 100.
     */
    void setLowerThreshold( const uint8_t lo_thres )
        { lower_thres_ = lo_thres; }

    /**
     * @brief Set the upper threshold.
     * Every cell with a value >= this threshold and <= 100 is occupied.
     * @param up_thres The new threshold from 0 to 100.
     */
    void setUpperThreshold( const uint8_t up_thres )
        { upper_thres_ = up_thres; }

    /* Abstract functions inherited from GridMap2d */

    uint8_t getValue( const unsigned int x, const unsigned int y ) const
        { return (uint8_t)map_->data[y*getWidth() + x]; }

    void setValue( const unsigned int x, const unsigned int y, const uint8_t value )
        { map_->data[y*getWidth() + x] = (int8_t)value; }

    unsigned int getWidth() const
        { return map_->info.width; }

    unsigned int getHeight() const
        { return map_->info.height; }

    double getResolution() const
        { return map_->info.resolution; }

    lib_path::Point2d getOrigin() const
        { return lib_path::Point2d( map_->info.origin.position.x, map_->info.origin.position.x ); }

    void setOrigin( const lib_path::Point2d& p ) {
        map_->info.origin.position.x = p.x;
        map_->info.origin.position.y = p.y;
    }

    bool isFree( const unsigned int x, const unsigned int y ) const {
        return getValue( x, y ) <= lower_thres_;
    }

    bool isOccupied( const unsigned int x, const unsigned int y ) const {
        uint8_t value = getValue( x, y );
        return value >= upper_thres_ && value <= 100;
    }

    bool isNoInformation( const unsigned int x, const unsigned int y ) const
        { return getValue( x, y ) > 100; }

    bool point2cell( const double px, const double py, unsigned int& x, unsigned int& y ) const {
        if (!isInMap( px, py ))
            return false;
        x = (int)((px - map_->info.origin.position.x)/map_->info.resolution);
        y = (int)((py - map_->info.origin.position.y)/map_->info.resolution);
        return true;
    }

    void cell2point( const unsigned int x, const unsigned int y, double& px, double& py ) const {
        px = map_->info.resolution*(double)x + map_->info.origin.position.x + 0.5*map_->info.resolution;
        py = map_->info.resolution*(double)y + map_->info.origin.position.y + 0.5*map_->info.resolution;
    }

    bool isInMap( const int x, const int y ) const
        { return !( x < 0 || x >= (int)getWidth() || y < 0 || y >= (int)getHeight()); }

    bool isInMap( const double x, const double y ) const {
        double mx = (int)((x - map_->info.origin.position.x)/map_->info.resolution);
        double my = (int)((y - map_->info.origin.position.y)/map_->info.resolution);
        return !( mx < 0 || mx >= getWidth() || my < 0 || my >= (int)getHeight());
    }

protected:
    /// The underlying ROS map
    nav_msgs::OccupancyGrid* map_;

    /// Lower threshold. Every cell with a value <= this threshold is free
    uint8_t lower_thres_;

    /// Upper threshold. Every cell with a value >= this threshold is occupied (or no information).
    uint8_t upper_thres_;
};

} // namespace lib_ros_util

#endif // OCCUPANCYGRIDWRAPPER_H
