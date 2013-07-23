#ifndef DOUBLE_BUFFER_H
#define DOUBLE_BUFFER_H

/// PROJECT
#include <designer/boxed_object.h>
#include <designer/connection_type.h>

/// SYSTEM
#include <QMutex>

namespace vision_evaluator {

class DoubleBuffer : public BoxedObject
{
    Q_OBJECT

public:
    DoubleBuffer();

    virtual void fill(QBoxLayout* layout);
    virtual void messageArrived(ConnectorIn* source);
    virtual void tick();

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

private:
    void swapBuffers();

private:
    ConnectorIn* input_;
    ConnectorOut* output_;

    QMutex mutex_;

    struct State : public Memento {
        ConnectionType::Ptr buffer_back_;
        ConnectionType::Ptr buffer_front_;
    };

    State state;
};

}

#endif // DOUBLE_BUFFER_H
