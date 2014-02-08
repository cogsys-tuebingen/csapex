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

    bool isProcessing();

public Q_SLOTS:
    void forwardMessage(Connectable* source);

    void forwardMessageDirectly(ConnectorIn* source);
    void forwardMessageSynchronized(ConnectorIn* source);

    void addInput(ConnectorIn* source);

    void eventGuiChanged();
    void tick();

    void triggerError(bool e, const std::string& what);

    void setSynchronizedInputs(bool s);
    bool isSynchronizedInputs() const;

    void addParameter(param::Parameter* param);
    void addParameterCallback(param::Parameter *param, boost::function<void(param::Parameter *)> cb);
    void addParameterCondition(param::Parameter* param, boost::function<bool()> enable_condition);

    void setProcessing(bool p);
    bool isProcessing() const;

Q_SIGNALS:
    void messageProcessed();

private:
    void parameterChanged(param::Parameter* param);
    void parameterChanged(param::Parameter* param, boost::function<void(param::Parameter *)> cb);

public:
    static const unsigned timer_history_length_;

private:
    Node* node_;

    bool synchronized_inputs_;
    std::map<ConnectorIn*, bool> has_msg_;

    QMutex changed_params_mutex_;
    std::vector<std::pair<param::Parameter*, boost::function<void(param::Parameter *)> > > changed_params_;
    std::map<param::Parameter*, boost::function<bool()> > conditions_;

    std::deque<TimerPtr> timer_history_;

    bool thread_initialized_;
    volatile bool is_processing_;
};

}

#endif // NODE_WORKER_H
