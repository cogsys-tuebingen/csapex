#ifndef FULCRUM_H
#define FULCRUM_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QObject>
#include <QPointF>

namespace csapex
{
class Fulcrum : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<Fulcrum> Ptr;

    enum Type {
        CURVE = 0,
        LINEAR = 1,
        HANDLE = LINEAR,
        OUT = 10,
        IN = 11
    };

public:
    Fulcrum(Connection* parent, const QPointF& p, int t);

    void move(const QPointF& pos, bool dropped);
    QPointF pos() const;

    int id() const;
    void setId(int id);

    int type() const;

    Connection* connection() const;

Q_SIGNALS:
    void moved(Fulcrum*, bool dropped);

private:
    Connection* parent_;
    int id_;
    int type_;
    QPointF pos_;
};
}
#endif // FULCRUM_H
