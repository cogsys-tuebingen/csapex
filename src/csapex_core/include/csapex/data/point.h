#ifndef POINT_H
#define POINT_H

/// COMPONENT
#include <csapex_core/csapex_core_export.h>

namespace csapex
{
struct CSAPEX_CORE_EXPORT Point
{
    float x;
    float y;

    Point(float x, float y) : x(x), y(y)
    {
    }

    Point() : x(0.f), y(0.f)
    {
    }

    bool operator==(const Point& rhs) const
    {
        return x == rhs.x && y == rhs.y;
    }
    bool operator!=(const Point& rhs) const
    {
        return x != rhs.x || y != rhs.y;
    }
};

}  // namespace csapex

#endif  // POINT_H
