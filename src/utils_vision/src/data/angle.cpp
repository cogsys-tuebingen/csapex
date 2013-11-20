/// HEADER
#include "angle.h"

/// SYSTEM
#include <cmath>

namespace MathHelper {
double NormalizeAngle(const double angle) {
  double ret = angle;
  while (ret > M_PI)
    ret -= 2.0 * M_PI;
  while (ret < -M_PI)
    ret += 2.0 * M_PI;
  return ret;
}
}

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
