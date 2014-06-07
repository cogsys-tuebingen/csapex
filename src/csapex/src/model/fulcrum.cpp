/// HEADER
#include <csapex/model/fulcrum.h>

/// COMPONENT
#include <csapex/model/connection.h>

using namespace csapex;


Fulcrum::Fulcrum(Connection* parent, const QPointF& p, int t)
    : parent_(parent), type_(t), pos_(p)
{}

void Fulcrum::move(const QPointF& pos, bool dropped)
{
    pos_ = pos;
    Q_EMIT moved(this, dropped);
}

QPointF Fulcrum::pos() const
{
    return pos_;
}

int Fulcrum::id() const
{
    return id_;
}

void Fulcrum::setId(int id)
{
    id_ = id;
}

int Fulcrum::type() const
{
    return type_;
}

Connection* Fulcrum::connection() const
{
    return parent_;
}
