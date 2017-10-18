#ifndef NODE_WORKER_H
#define NODE_WORKER_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/utility/utility_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/param/parameter.h>
#include <csapex/model/error_state.h>
#include <csapex/utility/uuid.h>
#include <csapex/utility/notification.h>
#include <csapex/model/execution_mode.h>
#include <csapex/model/observer.h>
#include <csapex/utility/notifier.h>
#include <csapex/model/activity_type.h>
#include <csapex/model/execution_state.h>

/// SYSTEM
#include <map>
#include <functional>
#include <mutex>
#include <vector>
#include <atomic>
#include <csapex/utility/slim_signal.hpp>
#include <boost/optional.hpp>

namespace csapex {

class ProfilerLocal;
class Interval;

class CSAPEX_EXPORT NodeWorker : public ErrorState, public Observer, public Notifier
{
public:
    typedef std::shared_ptr<NodeWorker> Ptr;

public:
    NodeWorker(NodeHandlePtr node_handle);
    ~NodeWorker();

    NodeHandlePtr getNodeHandle() const;
    NodePtr getNode() const;
    UUID getUUID() const;

    long getSequenceNumber() const;

    void initialize();
    void reset();

    void ioChanged();


    void triggerTryProcess();
    void triggerPanic();

    void setState(ExecutionState state);
    ExecutionState getExecutionState() const;

    std::shared_ptr<ProfilerLocal> getProfiler();

    bool isEnabled() const;
    bool isIdle() const;
    bool isProcessing() const;
    bool isFired() const;


    bool isProcessingEnabled() const;
    void setProcessingEnabled(bool e);

    void setProfiling(bool profiling);
    bool isProfiling() const;

    void setIOError(bool error);

    /* REMOVE => UI*/ void setMinimized(bool min);

    bool canExecute();
    bool canProcess() const;
    bool canReceive() const;
    bool canSend() const;
    bool areAllInputsAvailable() const;

    void trySendEvents();

public:
    bool startProcessingMessages();
    void forwardMessages(bool send_parameters);

    bool execute();

    void checkParameters();

    void triggerError(bool e, const std::string& what);

    void killExecution();

    void outgoingMessagesProcessed();

public:
    slim_signal::Signal<void()> destroyed;
    slim_signal::Signal<void()> panic;

    slim_signal::Signal<void(bool)> enabled;

    slim_signal::Signal<void()> execution_state_changed;


    slim_signal::Signal<void(NodeWorker* worker, ActivityType type, std::shared_ptr<const Interval> stamp)> interval_start;
    slim_signal::Signal<void(NodeWorker* worker, std::shared_ptr<const Interval> stamp)> interval_end;

    slim_signal::Signal<void(NodeWorker* worker)> start_profiling;
    slim_signal::Signal<void(NodeWorker* worker)> stop_profiling;

    slim_signal::Signal<void()> messages_processed;
    slim_signal::Signal<void()> processRequested;
    slim_signal::Signal<void()> try_process_changed;

private:
    void publishParameter(csapex::param::Parameter *p);
    void publishParameterOn(const csapex::param::Parameter &p, Output *out);

    void finishTimer(TimerPtr t);

    void signalExecutionFinished();
    void signalMessagesProcessed(bool processing_aborted = false);

    void updateState();

    void finishProcessing();

    void errorEvent(bool error, const std::string &msg, ErrorLevel level) override;

    void sendEvents(bool active);

    void connectConnector(ConnectablePtr c);
    void disconnectConnector(Connector *c);

    bool hasActiveOutputConnection();

private:
    mutable std::recursive_mutex sync;

    NodeHandlePtr node_handle_;

    mutable std::recursive_mutex current_exec_mode_mutex_;
    boost::optional<ExecutionMode> current_exec_mode_;

    bool is_setup_;
    ExecutionState state_;

    bool is_processing_;

    Event* trigger_process_done_;

    Event* trigger_activated_;
    Event* trigger_deactivated_;

    std::map<Connector*, std::vector<slim_signal::Connection>> port_connections_;

    mutable std::recursive_mutex state_mutex_;

    std::recursive_mutex timer_mutex_;

    //TimerPtr profiling_timer_;
    std::shared_ptr<ProfilerLocal> profiler_;

    long guard_;
};

}

#endif // NODE_WORKER_H
