/// HEADER
#include <csapex/model/fulcrum.h>

/// COMPONENT
#include <csapex/model/connection.h>

using namespace csapex;


Fulcrum::Fulcrum(int connection_id, const Point& p, int type, const Point &handle_in, const Point &handle_out)
    : connection_id_(connection_id), type_(type), pos_(p), handle_in_(handle_in), handle_out_(handle_out)
{}

Fulcrum::Fulcrum(const Fulcrum& copy)
    : connection_id_(copy.connection_id_), type_(copy.type_), pos_(copy.pos_), handle_in_(copy.handle_in_), handle_out_(copy.handle_out_)
{
}

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
    return connection_id_;
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
