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

/// SYSTEM
#include <map>
#include <functional>
#include <mutex>
#include <vector>
#include <atomic>
#include <csapex/utility/slim_signal.hpp>
#include <boost/optional.hpp>

namespace csapex {

class Profiler;
class Interval;

class CSAPEX_EXPORT NodeWorker : public ErrorState
{
public:
    enum ActivityType {
        TICK,
        PROCESS,
        SLOT,
        OTHER
    };

public:
    typedef std::shared_ptr<NodeWorker> Ptr;

    enum class State {
        IDLE,
        ENABLED,
        FIRED,
        PROCESSING
    };

public:
    NodeWorker(NodeHandlePtr node_handle);
    ~NodeWorker();

    NodeHandlePtr getNodeHandle();
    NodePtr getNode() const;
    UUID getUUID() const;

    long getSequenceNumber() const;

    void reset();

    void triggerTryProcess();
    void triggerPanic();

    void setState(State state);
    State getState() const;

    std::shared_ptr<Profiler> getProfiler();

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

    bool canProcess() const;
    bool canReceive() const;
    bool canSend() const;
    bool areAllInputsAvailable() const;

    void trySendEvents();

public:
    bool tick();

    void startProcessingMessages();
    void forwardMessages(bool send_parameters);

    bool tryProcess();

    void checkParameters();    
    void checkIO();

    void triggerError(bool e, const std::string& what);

    void killExecution();

    void sendMessages(bool ignore_sink);
    void outgoingMessagesProcessed();

public:
    csapex::slim_signal::Signal<void()> destroyed;
    csapex::slim_signal::Signal<void()> panic;

    csapex::slim_signal::Signal<void()> ticked;
    csapex::slim_signal::Signal<void(bool)> enabled;

    csapex::slim_signal::Signal<void(NodeWorker* worker, int type, std::shared_ptr<const Interval> stamp)> interval_start;
    csapex::slim_signal::Signal<void(NodeWorker* worker, std::shared_ptr<const Interval> stamp)> interval_end;

    csapex::slim_signal::Signal<void(NodeWorker* worker)> start_profiling;
    csapex::slim_signal::Signal<void(NodeWorker* worker)> stop_profiling;

    csapex::slim_signal::Signal<void(Notification)> notification;

    csapex::slim_signal::Signal<void()> messages_processed;
    csapex::slim_signal::Signal<void()> processRequested;
    csapex::slim_signal::Signal<void()> try_process_changed;

private:
    void publishParameters();
    void publishParameter(csapex::param::Parameter *p);
    void publishParameterOn(const csapex::param::Parameter &p, Output *out);

    void finishTimer(TimerPtr t);

    void signalExecutionFinished();
    void signalMessagesProcessed(bool processing_aborted = false);

    void activateOutput();

    void updateState();
    void updateTransitionConnections();

    void finishGenerator();
    void finishProcessing();

    void errorEvent(bool error, const std::string &msg, ErrorLevel level) override;

    void sendEvents(bool active);

    void connectConnector(ConnectablePtr c);
    void disconnectConnector(Connectable *c);

    bool hasActiveOutputConnection();

private:
    mutable std::recursive_mutex sync;

    NodeHandlePtr node_handle_;

    boost::optional<ExecutionMode> current_exec_mode_;

    bool is_setup_;
    State state_;

    Event* trigger_tick_done_;
    Event* trigger_process_done_;

    Event* trigger_activated_;
    Event* trigger_deactivated_;

    std::vector<csapex::slim_signal::ScopedConnection> connections_;
    std::map<Connectable*, std::vector<csapex::slim_signal::Connection>> port_connections_;

    int ticks_;

    mutable std::recursive_mutex state_mutex_;

    std::recursive_mutex timer_mutex_;

    //TimerPtr profiling_timer_;
    std::shared_ptr<Profiler> profiler_;

    long guard_;
};

}

#endif // NODE_WORKER_H
