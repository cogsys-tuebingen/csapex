#ifndef FULCRUM_H
#define FULCRUM_H

/// COMPONENT
#include <csapex/data/point.h>
#include <csapex/model/model_fwd.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <memory>
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{
class CSAPEX_EXPORT Fulcrum
{
public:
    typedef std::shared_ptr<Fulcrum> Ptr;

    enum Type {
		FULCRUM_CURVE = 0,
		FULCRUM_LINEAR = 1,
		FULCRUM_OUT = 10,
		FULCRUM_IN = 11,
		FULCRUM_HANDLE = FULCRUM_IN
    };

public:
    enum Handle {
        HANDLE_NONE = 0,
        HANDLE_IN = 1,
        HANDLE_OUT = 2,
        HANDLE_BOTH = 3
    };

public:
    Fulcrum(Connection* parent, const Point& p, int type, const Point& handle_in, const Point& handle_out);

    void move(const Point& pos, bool dropped);
    Point pos() const;

    void moveHandles(const Point& in, const Point& out, bool dropped);
    void moveHandleIn(const Point& pos, bool dropped);
    void moveHandleOut(const Point& pos, bool dropped);
    Point handleIn() const;
    Point handleOut() const;

    int id() const;
    void setId(int id);

    int connectionId() const;

    int type() const;
    void setType(int type);

    Connection* connection() const;

public:
    csapex::slim_signal::Signal<void (Fulcrum*, bool dropped)> moved;
    csapex::slim_signal::Signal<void (Fulcrum*, bool dropped, int no)> movedHandle;
    csapex::slim_signal::Signal<void (Fulcrum*, int type)> typeChanged;

private:
    Connection* parent_;
    int id_;
    int type_;
    Point pos_;
    Point handle_in_;
    Point handle_out_;
};
}
#endif // FULCRUM_H
