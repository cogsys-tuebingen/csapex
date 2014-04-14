#ifndef DOUBLE_BUFFER_H
#define DOUBLE_BUFFER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>

/// SYSTEM
#include <QMutex>

namespace csapex {

class DoubleBuffer : public Node
{
public:
    DoubleBuffer();

    virtual void setup();
    virtual void process();
    virtual void tick();

    virtual QIcon getIcon() const;

protected:
    void checkIfDone();

private:
    void swapBuffers();

private:
    ConnectorIn* input_;
    ConnectorOut* output_;

    QMutex mutex_;

    bool dirty_;

    ConnectionType::Ptr buffer_back_;
    ConnectionType::Ptr buffer_front_;
};

}

#endif // DOUBLE_BUFFER_H
