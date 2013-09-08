#ifndef RELAY_H
#define RELAY_H

/// PROJECT
#include <csapex/boxed_object.h>
#include <csapex/connection_type.h>

/// SYSTEM
#include <QMutex>

namespace csapex {

class Relay : public BoxedObject
{
    Q_OBJECT

public:
    Relay();

    virtual void fill(QBoxLayout* layout);
    virtual void messageArrived(ConnectorIn* source);

private:
    ConnectorIn* input_;
    ConnectorOut* output_;

    QMutex mutex_;
};

}

#endif // RELAY_H
