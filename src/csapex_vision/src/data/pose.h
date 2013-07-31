#ifndef POSE_H
#define POSE_H

/// SYSTEM
#include <Eigen/Geometry>

/**
 * @brief The Pose class represents a position and an orientation
 */
class Pose
{
public:
    /**
     * @brief Pose
     */
    Pose();

    /**
     * @brief Pose
     * @param position
     * @param orientation
     */
    Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

public:
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

#endif // POSE_H
