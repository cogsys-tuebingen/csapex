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

/// SYSTEM
#include <map>
#include <functional>
#include <mutex>
#include <vector>
#include <atomic>
#include <csapex/utility/slim_signal.hpp>

namespace csapex {

class NodeWorker : public ErrorState
{
public:
    enum ActivityType {
        TICK,
        PROCESS,
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

    void triggerCheckTransitions();
    void triggerPanic();

    void setState(State state);
    State getState() const;

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

    bool isWaitingForTrigger() const;
    bool canProcess() const;
    bool canReceive() const;
    bool canSend() const;
    bool areAllInputsAvailable() const;

    std::vector<TimerPtr> extractLatestTimers();

public:
    bool tick();

    void startProcessingMessages();
    void forwardMessages(bool send_parameters);

    void checkTransitions();

    void checkParameters();    
    void checkIO();

    void triggerError(bool e, const std::string& what);

    void killExecution();

    void sendMessages();
    void notifyMessagesProcessed();

public:
    csapex::slim_signal::Signal<void()> panic;

    csapex::slim_signal::Signal<void()> ticked;
    csapex::slim_signal::Signal<void(bool)> enabled;

    csapex::slim_signal::Signal<void(NodeWorker* worker, int type, long stamp)> timerStarted;
    csapex::slim_signal::Signal<void(NodeWorker* worker, long stamp)> timerStopped;

    csapex::slim_signal::Signal<void(NodeWorker* worker)> startProfiling;
    csapex::slim_signal::Signal<void(NodeWorker* worker)> stopProfiling;

    csapex::slim_signal::Signal<void()> threadChanged;
    csapex::slim_signal::Signal<void(bool)> errorHappened;

    csapex::slim_signal::Signal<void()> messages_processed;
    csapex::slim_signal::Signal<void()> processRequested;
    csapex::slim_signal::Signal<void()> checkTransitionsRequested;

private:
    void publishParameters();
    void publishParameter(csapex::param::Parameter *p);
    void publishParameterOn(const csapex::param::Parameter &p, Output *out);

    void finishTimer(TimerPtr t);

    void signalExecutionFinished();
    void signalMessagesProcessed();

    void activateOutput();
    void updateTransitionConnections();

    void finishGenerator();
    void finishProcessing();

    void errorEvent(bool error, const std::string &msg, ErrorLevel level) override;


    void connectConnector(Connectable *c);
    void disconnectConnector(Connectable *c);

    void checkTransitionsImpl(bool try_fire);

private:
    mutable std::recursive_mutex sync;

    NodeHandlePtr node_handle_;

    bool is_setup_;
    State state_;

    Trigger* trigger_tick_done_;
    Trigger* trigger_process_done_;

    std::vector<csapex::slim_signal::Connection> handle_connections_;
    std::map<Connectable*, std::vector<csapex::slim_signal::Connection>> connections_;

    int ticks_;

    mutable std::recursive_mutex state_mutex_;

    std::recursive_mutex timer_mutex_;
    std::vector<TimerPtr> timer_history_;

    std::atomic<bool> profiling_;
    TimerPtr current_process_timer_;
};

}

#endif // NODE_WORKER_H
