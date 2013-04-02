#ifndef ANGLE_H
#define ANGLE_H

#include <Eigen/Geometry>

/**
 * @brief The Angle class helps at calculating with angles
 */
class Angle
{
public:
    /**
     * @brief Angle
     * @param radians angle in radians
     */
    Angle(double radians = 0);

    /**
     * @brief fromDegrees static factory function
     * @param degrees angle in degrees
     * @return
     */
    static Angle fromDegrees(double degrees);


    /**
     * @brief toDegrees
     * @return
     */
    double toDegrees() const;

    /**
     * @brief toRadians
     * @return
     */
    double toRadians() const;

    /**
     * @brief toQuaternion
     * @param axis the axis around to rotate with this angle
     * @return
     */
    Eigen::Quaterniond toQuaternion(const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ()) const;

public:
    /**
     * @brief quatToRPY Computes Euler angles from a quaternion
     * @param q
     * @param roll
     * @param pitch
     * @param yaw
     */
    static inline void quatToRPY(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
        const double q0 = q.w();
        const double q1 = q.x();
        const double q2 = q.y();
        const double q3 = q.z();
        roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
        pitch = asin(2*(q0*q2-q3*q1));
        yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
    }

private:
    double value_;
};

#endif // ANGLE_H
