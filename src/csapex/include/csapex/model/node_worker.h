#ifndef NODE_WORKER_H
#define NODE_WORKER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_param/parameter.h>

/// SYSTEM
#include <QObject>
#include <QMutex>
#include <map>
#include <deque>
#include <boost/function.hpp>

namespace csapex {

struct NodeWorker : public QObject, public param::Parameter::access
{
    Q_OBJECT

    friend class ProfilingWidget;

public:
    typedef boost::shared_ptr<NodeWorker> Ptr;

public:
    NodeWorker(Node* node);

public Q_SLOTS:
    void forwardMessage(Connector* source);

    void forwardMessageDirectly(ConnectorIn* source);
    void forwardMessageSynchronized(ConnectorIn* source);

    void addInput(ConnectorIn* source);

    void eventGuiChanged();
    void tick();

    void triggerError(bool e, const std::string& what);

    void setSynchronizedInputs(bool s);

    void addParameterCallback(const param::Parameter::Ptr& param, boost::function<void(param::Parameter *)> cb);

Q_SIGNALS:
    void messageProcessed();

private:
    void parameterChanged(param::Parameter* param, boost::function<void(param::Parameter *)> cb);

public:
    static const unsigned timer_history_length_;

private:
    Node* node_;

    bool synchronized_inputs_;
    std::map<ConnectorIn*, bool> has_msg_;

    QMutex changed_params_mutex_;
    std::vector<std::pair<param::Parameter*, boost::function<void(param::Parameter *)> > > changed_params_;

    std::deque<int> timer_history_;
};

}

#endif // NODE_WORKER_H
