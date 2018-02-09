#ifndef SUBPROCESS_NODE_WORKER_H
#define SUBPROCESS_NODE_WORKER_H

/// COMPONENT
#include <csapex/model/node_worker.h>

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/utility/utility_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/param/parameter.h>
#include <csapex/model/error_state.h>
#include <csapex/utility/uuid.h>
#include <csapex/model/notification.h>
#include <csapex/model/execution_mode.h>
#include <csapex/model/observer.h>
#include <csapex/model/notifier.h>
#include <csapex/model/activity_type.h>
#include <csapex/model/execution_state.h>
#include <csapex/model/activity_modifier.h>
#include <csapex/utility/subprocess.h>

/// SYSTEM
#include <map>
#include <functional>
#include <mutex>
#include <vector>
#include <atomic>
#include <future>
#include <csapex/utility/slim_signal.hpp>


namespace csapex {

class ProfilerImplementation;
class Interval;


class CSAPEX_EXPORT SubprocessNodeWorker : public NodeWorker
{
private:
    using Lock = boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex>;

public:
    SubprocessNodeWorker(NodeHandlePtr node_handle);
    ~SubprocessNodeWorker();

    void initialize() override;
    void processNode() override;



protected:
    void startSubprocess(const SubprocessChannel::MessageType type);
    void finishSubprocess();

    void finishProcessing() override;
    void handleChangedParametersImpl(const Parameterizable::ChangedParameterList& changed_params) override;

private:
    void runSubprocessLoop();

    void handleParameterUpdate(const SubprocessChannel::Message& msg);

    void handleProcessParent(const SubprocessChannel::Message &msg);
    void handleProcessChild(const SubprocessChannel::Message& msg);
    void finishHandleProcessChild();

    void transmitParameter(const param::ParameterPtr& p);

private:
    pid_t pid_;

    Subprocess subprocess_;

    std::vector<param::Parameter*> changed_parameters_;

    std::future<void> async_future_;
};

}

#endif // SUBPROCESS_NODE_WORKER_H
