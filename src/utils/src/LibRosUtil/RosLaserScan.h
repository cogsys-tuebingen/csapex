/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Marks
 @date 2011

 @file RosLaserScan.h
 */

#ifndef ROSLASERSCAN_H
#define ROSLASERSCAN_H

// C/C++
#include <vector>

// Eigen
#include <Eigen/Core>
#include <Eigen/StdVector>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

// Project
#include <utils/LibLaserProcessing/laser_beam.h>

namespace lib_ros_util {

/// @todo Add a method to transform the scan into a given fram using tf

/**
 * Converts a ROS laser scan to a vector of LaserBeam objects. Computes
 * the yaw angle and the obstacle position for each beam and
 * validates the range reading.
 */
class RosLaserScan {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Create object, set all members to zero.
     */
    RosLaserScan();

    /**
     * Process new laser scan.
     *
     * @param scan The new range readings.
     * @param tf Transform laser to robot base link.
     *
     * @return False if there was an error.
     */
    bool update( const sensor_msgs::LaserScan::ConstPtr& scan );

    /**
     * Return the range readings.
     * @attention Check if the returned objects are valid!
     *
     * @return Vector containig all range readings.
     */
    const std::vector<lib_laser_processing::LaserBeam>& getBeams() const {
        return beams_;
    }

private:

    /**
     * Check minimum and maximum angle settings as well as minimum and maximum
     * range. Recompute beam yaw angles, sinus and cosinus etc. if necessary.
     *
     * @param scan New laser scan.
     *
     * @return True if parameters have changed.
     */
    bool checkSettings( const sensor_msgs::LaserScan::ConstPtr& scan );

    /// Laser scanners minimum yaw angle [rad]
    float angle_min_;

    /// Laser scanners maximum yaw angle [rad]
    float angle_max_;

    /// Laser scanners minimum (valid) range [m]
    float range_min_;

    /// Laser scanners maximum (valid) range [m]
    float range_max_;

    /// Current range readings
    std::vector<lib_laser_processing::LaserBeam> beams_;

    /// Cosinus of beams yaw angles
    std::vector<double> cos_;

    /// Sinus of beams yaw angles
    std::vector<double> sin_;

    /// Flag if we obtained at least one laser scan
    bool scan_obtained_;

    /// Flag if we got a laser to base link transform
    bool tf_obtained_;
};

} // namespace

#endif // ROSLASERSCAN_H
