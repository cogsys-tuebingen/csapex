#ifndef FULCRUM_H
#define FULCRUM_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <memory>
#include <QObject>
#include <QPointF>

namespace csapex
{
class Fulcrum : public QObject
{
    Q_OBJECT

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
    Fulcrum(Connection* parent, const QPointF& p, int type, const QPointF& handle_in, const QPointF& handle_out);

    void move(const QPointF& pos, bool dropped);
    QPointF pos() const;

    void moveHandles(const QPointF& in, const QPointF& out, bool dropped);
    void moveHandleIn(const QPointF& pos, bool dropped);
    void moveHandleOut(const QPointF& pos, bool dropped);
    QPointF handleIn() const;
    QPointF handleOut() const;

    int id() const;
    void setId(int id);

    int connectionId() const;

    int type() const;
    void setType(int type);

    Connection* connection() const;

Q_SIGNALS:
    void moved(Fulcrum*, bool dropped);
    void movedHandle(Fulcrum*, bool dropped, int no);
    void typeChanged(Fulcrum*, int type);

private:
    Connection* parent_;
    int id_;
    int type_;
    QPointF pos_;
    QPointF handle_in_;
    QPointF handle_out_;
};
}
#endif // FULCRUM_H
