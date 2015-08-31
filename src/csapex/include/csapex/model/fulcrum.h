#ifndef FULCRUM_H
#define FULCRUM_H

/// COMPONENT
#include <csapex/data/point.h>
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <memory>
#include <boost/signals2/signal.hpp>

namespace csapex
{
class Fulcrum
{
public:
    typedef std::shared_ptr<Fulcrum> Ptr;

    enum Type {
        CURVE = 0,
        LINEAR = 1,
        OUT = 10,
        IN = 11,
        HANDLE = IN
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
    boost::signals2::signal<void (Fulcrum*, bool dropped)> moved;
    boost::signals2::signal<void (Fulcrum*, bool dropped, int no)> movedHandle;
    boost::signals2::signal<void (Fulcrum*, int type)> typeChanged;

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
