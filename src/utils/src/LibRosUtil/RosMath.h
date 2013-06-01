/**
 * @file RosMath.h
 * @date Jan 2012
 * @author marks
 */

#ifndef ROSMATH_H
#define ROSMATH_H

// C/C++
#include <cmath>

// ROS
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

namespace lib_ros_util {

/**
 * @brief A collection of math functions working with ROS types.
 */
class RosMath
{
public:

    /**
     * @brief Calculate the length of a 3D vector.
     * @param p The vector.
     * @param Length of the vector.
     */
    static inline double norm( const geometry_msgs::Point& p ) {
        return sqrt( p.x*p.x + p.y*p.y + p.z*p.z );
    }

    /**
     * @brief Calculate the length of a 3D vector and neglects the z coordinate.
     * @param p The vector.
     * @param Length of the 2D part of the vector.
     */
    static inline double norm2d( const geometry_msgs::Point& p ) {
        return sqrt( p.x*p.x + p.y*p.y );
    }

    /**
     * @brief Calculate the distance betweent two points.
     * @param p Point one.
     * @param q Point two.
     * @return The distance.
     */
    static inline double distance( const geometry_msgs::Point& p, const geometry_msgs::Point& q ) {
        return sqrt( pow( p.x - q.x, 2 ) + pow( p.y - q.y, 2 ) + pow( p.z - q.z, 2 ));
    }

    /**
     * @brief Calculate the distance betweent two points (neglects z coordinate).
     * @param p Point one.
     * @param q Point two.
     * @return The 2D distance.
     */
    static inline double distance2d( const geometry_msgs::Point& p, const geometry_msgs::Point& q ) {
        return sqrt( pow( p.x - q.x, 2 ) + pow( p.y - q.y, 2 ));
    }

    /**
     * @brief Calculate the difference in yaw.
     * @param p Pose one.
     * @param q Pose q.
     * @return Yaw difference p - q [-PI, PI].
     */
    static inline double angle2d( const geometry_msgs::Quaternion& p, const geometry_msgs::Quaternion& q ) {
        double d_yaw = tf::getYaw( p ) - tf::getYaw( q );
        while ( d_yaw > M_PI ) d_yaw -= 2.0*M_PI;
        while ( d_yaw < -M_PI ) d_yaw += 2.0*M_PI;
        return d_yaw;
    }

    /**
     * @brief Calculate 2D position and yaw difference.
     * @param p Pose one.
     * @param q Pose two.
     * @param d_dist Will hold the 2D distance.
     * @param d_angle Will hold the yaw difference [0,PI].
     */
    static inline void poseDelta2d( const geometry_msgs::Pose& p, const geometry_msgs::Pose& q, double& d_dist, double& d_angle ) {
        d_dist = distance( p.position, q.position );
        d_angle = fabs( angle2d( p.orientation, q.orientation ));
    }

    /**
     * @brief Check if position and angle difference are less than given values.
     * @param p Pose one
     * @param q Pose two
     * @param max_d_dist Maximum distance.
     * @param max_d_angle Maximum angle difference.
     */
    static inline bool isEqual2d( const geometry_msgs::Pose& p, const geometry_msgs::Pose& q, double max_d_dist, double max_d_angle ) {
        double d_dist, d_angle;
        poseDelta2d( p, q, d_dist, d_angle );
        return (d_dist <= max_d_dist) && (d_angle <= max_d_angle);
    }

};

} // namespace

#endif // ROSMATH_H
