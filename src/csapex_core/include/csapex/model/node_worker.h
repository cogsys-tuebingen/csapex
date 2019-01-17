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
#include <csapex/utility/thread_debug_helper.hpp>
#include <csapex/model/notification.h>
#include <csapex/model/execution_mode.h>
#include <csapex/model/observer.h>
#include <csapex/model/notifier.h>
#include <csapex/model/tracing_type.h>
#include <csapex/model/execution_state.h>
#include <csapex/model/activity_modifier.h>
#include <csapex/model/parameterizable.h>

/// SYSTEM
#include <map>
#include <functional>
#include <mutex>
#include <vector>
#include <atomic>
#include <csapex/utility/slim_signal.hpp>
#include <boost/optional.hpp>
#include <thread>

namespace csapex
{
class ProfilerImplementation;
class Interval;

class CSAPEX_CORE_EXPORT NodeWorker : public ErrorState, public Observer, public Notifier, private ThreadDebugHelper
{
public:
    typedef std::shared_ptr<NodeWorker> Ptr;

public:
    ~NodeWorker();

    virtual void initialize();
    void reset();
    void handleChangedParameters();

    bool startProcessingMessages();
    bool startProcessingSlot(const SlotWeakPtr& slot);

    NodeHandlePtr getNodeHandle() const;
    NodePtr getNode() const;
    UUID getUUID() const;
    std::shared_ptr<ProfilerImplementation> getProfiler();
    ExecutionState getExecutionState() const;

    long getSequenceNumber() const;

    bool isProcessingEnabled() const;
    void setProcessingEnabled(bool e);

    void setProfiling(bool profiling);
    bool isProfiling() const;

    bool isEnabled() const;
    bool isIdle() const;
    bool isProcessing() const;

    bool canExecute();
    bool canProcess() const;
    bool canReceive() const;
    bool canSend() const;

    void notifyMessagesProcessedDownstream();

public:
    slim_signal::Signal<void()> destroyed;

    slim_signal::Signal<void(bool)> enabled;

    slim_signal::Signal<void(NodeWorker* worker, TracingType type, std::shared_ptr<const Interval> stamp)> interval_start;
    slim_signal::Signal<void(NodeWorker* worker, std::shared_ptr<const Interval> stamp)> interval_end;

    slim_signal::Signal<void(NodeWorker* worker)> start_profiling;
    slim_signal::Signal<void(NodeWorker* worker)> stop_profiling;

    slim_signal::Signal<void()> outgoing_messages_processed;
    slim_signal::Signal<void()> messages_processed;
    slim_signal::Signal<void()> processRequested;
    slim_signal::Signal<void()> try_process_changed;

protected:
    NodeWorker(NodeHandlePtr node_handle);

    virtual void handleChangedParametersImpl(const Parameterizable::ChangedParameterList& changed_params);

    virtual void processNode();
    virtual void finishProcessing();

    virtual void processSlot(const SlotWeakPtr& slot);

private:
    bool hasNode() const;
    void triggerTryProcess();

    void ioChanged();

    void updateParameterValues();

    void publishParameters();
    void publishParameter(csapex::param::Parameter* p);
    void publishParameterOn(const csapex::param::Parameter& p, Output* out);

    void finishTimer(TimerPtr t);

    void signalExecutionFinished();
    void signalMessagesProcessed(bool processing_aborted);

    void updateState();

    void pruneExecution();
    void skipExecution();

    void setProcessing(bool processing);

    void errorEvent(bool error, const std::string& msg, ErrorLevel level) override;

    void forwardMessages();
    void sendEvents(bool active);

    void connectConnector(ConnectablePtr c);
    void disconnectConnector(Connector* c);

    bool allInputsArePresent();

    std::vector<ActivityModifier> getIncomingActivityModifiers();
    void applyActivityModifiers(std::vector<ActivityModifier> activity_modifiers);

    connection_types::MarkerMessageConstPtr getFirstMarkerMessage();
    bool processMarker(const connection_types::MarkerMessageConstPtr& marker);

    void rememberExecutionMode();

    void startProfilerInterval(TracingType type);
    void stopActiveProfilerInterval();

protected:
    mutable std::recursive_mutex sync;

    NodeHandlePtr node_handle_;

    mutable std::recursive_mutex current_exec_mode_mutex_;
    boost::optional<ExecutionMode> current_exec_mode_;

    bool is_setup_;

private:
    mutable std::recursive_mutex state_mutex_;
    bool is_processing_;

    Event* trigger_process_done_;
    Event* trigger_activated_;
    Event* trigger_deactivated_;

    Slot* slot_enable_;
    Slot* slot_disable_;

    std::map<Connector*, std::vector<slim_signal::Connection>> port_connections_;

    std::recursive_mutex timer_mutex_;

    // TimerPtr profiling_timer_;
    std::shared_ptr<ProfilerImplementation> profiler_;

    long guard_;
};

}  // namespace csapex

#endif  // NODE_WORKER_H
