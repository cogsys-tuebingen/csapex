/// HEADER
#include <csapex/model/fulcrum.h>

/// COMPONENT
#include <csapex/model/connection.h>

using namespace csapex;


Fulcrum::Fulcrum(Connection* parent, const Point& p, int type, const Point &handle_in, const Point &handle_out)
    : parent_(parent), type_(type), pos_(p), handle_in_(handle_in), handle_out_(handle_out)
{}

void Fulcrum::move(const Point& pos, bool dropped)
{
    pos_ = pos;
    moved(this, dropped);
}

Point Fulcrum::pos() const
{
    return pos_;
}

void Fulcrum::moveHandles(const Point& in, const Point& out, bool dropped)
{
    if(in != handle_in_ || out != handle_out_ || dropped) {
        handle_in_ = in;
        handle_out_ = out;
        movedHandle(this, dropped, Fulcrum::HANDLE_BOTH);
    }
}


void Fulcrum::moveHandleIn(const Point& pos, bool dropped)
{
    if(pos != handle_in_ || dropped) {
        handle_in_ = pos;
        movedHandle(this, dropped, Fulcrum::HANDLE_IN);
    }
}

Point Fulcrum::handleIn() const
{
    return handle_in_;
}

void Fulcrum::moveHandleOut(const Point& pos, bool dropped)
{
    if(pos != handle_out_ || dropped) {
        handle_out_ = pos;
        movedHandle(this, dropped, Fulcrum::HANDLE_OUT);
    }
}

Point Fulcrum::handleOut() const
{
    return handle_out_;
}

int Fulcrum::id() const
{
    return id_;
}

int Fulcrum::connectionId() const
{
    return parent_->id();
}

void Fulcrum::setId(int id)
{
    id_ = id;
}

int Fulcrum::type() const
{
    return type_;
}

void Fulcrum::setType(int type)
{
    type_ = type;
    typeChanged(this, type);
}

Connection* Fulcrum::connection() const
{
    return parent_;
}
