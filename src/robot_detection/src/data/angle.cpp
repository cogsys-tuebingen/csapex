/// HEADER
#include "angle.h"

/// SYSTEM
#include <cmath>
#include <utils/LibUtil/MathHelper.h>

Angle::Angle(double radians)
    : value_(MathHelper::NormalizeAngle(radians))
{
}

double Angle::toDegrees() const
{
    return value_ * 180.0 / M_PI;
}


double Angle::toRadians() const
{
    return value_;
}

Angle Angle::fromDegrees(double degrees)
{
    return Angle(degrees / 180.0 * M_PI);
}

Eigen::Quaterniond Angle::toQuaternion(const Eigen::Vector3d& axis) const
{
    Eigen::Quaterniond result;
    result = Eigen::AngleAxisd(value_, axis);
    return result;
}
