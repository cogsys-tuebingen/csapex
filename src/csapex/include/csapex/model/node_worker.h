#ifndef NODE_WORKER_H
#define NODE_WORKER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QObject>
#include <map>
#include <deque>

namespace csapex {

struct NodeWorker : public QObject
{
    Q_OBJECT

    friend class ProfilingWidget;

public:
    typedef boost::shared_ptr<NodeWorker> Ptr;

public:
    NodeWorker(NodePtr node);

public Q_SLOTS:
    void forwardMessage(Connector* source);

    void forwardMessageDirectly(ConnectorIn* source);
    void forwardMessageSynchronized(ConnectorIn* source);

    void addInput(ConnectorIn* source);

    void eventGuiChanged();
    void tick();

    void triggerError(bool e, const std::string& what);

    void setSynchronizedInputs(bool s);

Q_SIGNALS:
    void messageProcessed();

public:
    static const unsigned timer_history_length_;

private:
    NodePtr node_;

    bool synchronized_inputs_;
    std::map<ConnectorIn*, bool> has_msg_;

    std::deque<int> timer_history_;
};

}

#endif // NODE_WORKER_H
