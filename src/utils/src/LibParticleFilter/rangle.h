#ifndef R_ANGLE
#define R_ANGLE


#include <math.h>

// this class represents angles in a sense of direction or orientation, not an enclosing between two arms
class Angle {
public:
    Angle()
        : angle(0.0) {
    }

    Angle(double ang)
        : angle(ang) {
        normalize();
    }

    Angle(double ang, int unit)
        : angle(ang) { // todo: avoid unnecessary init, if possible
        if (unit == 1) // degree
            angle = ang*M_PI/180.0;

        normalize();
    }

    Angle& operator+=(const Angle &rhs) {
        this->angle += rhs.angle;

        this->normalize();
        return *this;
    }

    const Angle operator+(const Angle &other) const {
      return Angle(*this) += other;
    }

    // normalize to 0..2*pi
    void normalize() {
        double r = floor(angle/(2.0*M_PI));
        angle -= r*2.0*M_PI;
    }

    double getDegrees() {
        return angle*180.0/M_PI;
    }

    void setDegrees(double deg) {
        angle = deg/180.0*M_PI;

        normalize();
    }

    double shortest_angular_distance(Angle to)
    {
        this->normalize();
        to.normalize();
        Angle result(to - angle);
        result.normalize();

      if (result > M_PI)
        // If the result > 180 deg,
        // It's shorter the other way.
        result = 2.0*M_PI - result;

      return result;
    }

    // a range is defined by two angles. isInRange returns true if and only if
    // the angle is clockwise of left and counterclockwise of right
    // In other words, if you travel clockwise and you get the sequence
    // ..., right, left, this, right, left, this, ...,
    // then isInRange returns true, otherwise false
    bool isInRange(Angle left, Angle right) {
        left.normalize();
        right.normalize();
        this->normalize();

        if (left < right)
            left.angle += 2*M_PI;

        if (*this < right)
            angle += 2*M_PI;

        return (*this < left);
    }

    operator double() {
        return angle;
    }

public: //todo: make private
    double angle;
};

#endif // R_ANGLE
