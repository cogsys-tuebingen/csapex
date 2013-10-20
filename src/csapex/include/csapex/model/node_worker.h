#ifndef NODE_WORKER_H
#define NODE_WORKER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QObject>

namespace csapex {

struct NodeWorker : public QObject
{
    Q_OBJECT

public:
    NodeWorker(Box* parent);

public Q_SLOTS:
    void forwardMessage(Connector* source);

    void forwardMessageDirectly(ConnectorIn* source);
    void forwardMessageSynchronized(ConnectorIn* source);

    void eventGuiChanged();
    void tick();

    void triggerError(bool e, const std::string& what);

private:
    BoxedObjectPtr node_;
    Box* parent_;
};

}

#endif // NODE_WORKER_H
