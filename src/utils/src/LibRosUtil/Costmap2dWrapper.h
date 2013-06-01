/**
 * @file Costmap2dWrapper.h
 * @date Jan 2012
 * @author marks
 */

#ifndef COSTMAP2DWRAPPER_H
#define COSTMAP2DWRAPPER_H

// ROS
#include <costmap_2d_extended/costmap_2d.h>

// Workspace
#include <utils/LibPath/common/GridMap2d.h>

namespace lib_ros_util {

/**
 * @class Costmap2dWrapper
 * @brief A ROS Costmap2d wrapper.
 *
 * This class makes it possible to use a ROS costmap implementation as input for
 * algorithms that use the ROS independend map interface from utils/LibPath.
 *
 * @attention At the current state not all member functions of this class are
 *      reviewed and fully tested. Use it with care!
 */
class Costmap2dWrapper : public lib_path::GridMap2d
{
public:

    /**
     * @brief Create a ROS Costmap2d wrapper.
     * The lower threshold defaults to 20, the upper one to 200.
     * @param map The wrapped costmap.
     */
    Costmap2dWrapper( costmap_2d::Costmap2D* map );

    virtual ~Costmap2dWrapper()
        { /* Nothing to do */ }

    /**
     * @brief Setter for the wrapped costmap object.
     * @attention There is no local copy of the costmap! The given pointer should be
     *      valid as long as this wrapper is in use.
     * @param newmap Points to the new underlying costmap.
     */
    void setCostmap( costmap_2d::Costmap2D* newmap )
        { costmap_ = newmap; }

    /**
     * @brief Set the lower threshold.
     * Every cell with a value less or equal to the given threshold is obstacle free.
     * @param thres The new lower threshold
     */
    void setLowerThreshold( const uint8_t thres )
        { lower_thres_ = thres; }

    /**
     * @brief Set the upper threshold.
     * Every cell with a value greater or equal to this threshold represents an obstacle.
     * This is not true for cell thats contain no information (value 255).
     * @param thres The new upper threshold.
     */
    void setUpperThreshold( const uint8_t thres )
        { upper_thres_ = thres; }


    /* Abstract functions inherited from GridMap2d */

    uint8_t getValue( const unsigned int x, const unsigned int y ) const
        { return (unsigned char)costmap_->getCost( x,y ); }

    void setValue( const unsigned int x, const unsigned int y, const uint8_t value )
        { return costmap_->setCost( x, y, (unsigned char)value ); }

    unsigned int getWidth() const
        { return costmap_->getSizeInCellsX(); }

    unsigned int getHeight() const
        { return costmap_->getSizeInCellsY(); }

    double getResolution() const
        { return costmap_->getResolution(); }

    lib_path::Point2d getOrigin() const
        { return lib_path::Point2d( costmap_->getOriginX(), costmap_->getOriginY()); }

    void setOrigin( const lib_path::Point2d& p ) /// @todo Think about this!
        { costmap_->updateOrigin( p.x, p.y ); }


    bool isFree( const unsigned int x, const unsigned int y ) const
        { return getValue( x,y ) <= lower_thres_; }

    bool isOccupied( const unsigned int x, const unsigned int y ) const {
        uint8_t value = getValue( x,y );
        return value >= upper_thres_ && value < costmap_2d::NO_INFORMATION;
    }

    bool isNoInformation( const unsigned int x, const unsigned int y ) const
        { return costmap_->getCost( x,y ) == costmap_2d::NO_INFORMATION; }

    bool point2cell( const double px, const double py, unsigned int& x, unsigned int& y ) const
        { return costmap_->worldToMap( px, py, x, y ); }

    void cell2point( const unsigned int x, const unsigned int y, double& px, double& py ) const
        { costmap_->mapToWorld( x, y, px, py ); }

    bool isInMap( const int x, const int y ) const
        { return !( x < 0 || x >= (int)getWidth() || y < 0 || y > (int)getHeight()); }

    bool isInMap( const double x, const double y ) const {
        unsigned int mx, my;
        return point2cell( x, y, mx, my ); /// @todo this is a hack
    }

private:
    /// Points to the underlying costmap
    costmap_2d::Costmap2D* costmap_;

    /// Lower threshold. Every cell with a value less or equal this threshold is free
    uint8_t lower_thres_;

    /** Upper threshold. Every cell with a value greater or equal to this threshold is occupied
     * (or it contains no information). */
    uint8_t upper_thres_;
};

} // namespace lib_ros_util

#endif // COSTMAP2DWRAPPER_H
