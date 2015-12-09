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
#include <csapex/model/node_handle.h>

/// SYSTEM
#include <map>
#include <functional>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <atomic>
#include <boost/signals2/signal.hpp>

namespace csapex {

class NodeWorker :
        public NodeHandle,
        public ErrorState
{
    friend class Node;
    friend class NodeBox;

public:
    enum ActivityType {
        TICK,
        PROCESS,
        OTHER
    };

public:
    typedef std::shared_ptr<NodeWorker> Ptr;

    static const double DEFAULT_FREQUENCY;

    enum class State {
        IDLE,
        ENABLED,
        FIRED,
        PROCESSING
    };

public:
    NodeWorker(const std::string& type, const UUID& uuid, NodePtr node);
    ~NodeWorker();

    void setNodeState(NodeStatePtr memento);
    NodeStatePtr getNodeState();
    NodeStatePtr getNodeStateCopy() const;


    void stop();
    void reset();

    virtual void triggerCheckTransitions() override;
    void triggerPanic();

    void setState(State state);
    State getState() const;

    virtual bool isEnabled() const override;
    bool isIdle() const;
    bool isProcessing() const;
    bool isFired() const;


    bool isProcessingEnabled() const;
    void setProcessingEnabled(bool e);

    void setProfiling(bool profiling);
    bool isProfiling() const;

    void setIOError(bool error);

    /* REMOVE => UI*/ void setMinimized(bool min);

    Input* addInput(ConnectionTypePtr type, const std::string& label, bool dynamic, bool optional);
    void addInput(InputPtr in);
    bool isParameterInput(Input* in) const;

    Output* addOutput(ConnectionTypePtr type, const std::string& label, bool dynamic);
    void addOutput(OutputPtr out);
    bool isParameterOutput(Output* out) const;

    Slot* addSlot(const std::string& label, std::function<void ()> callback, bool active);
    void addSlot(SlotPtr s);

    Trigger* addTrigger(const std::string& label);
    void addTrigger(TriggerPtr t);

    Connectable* getConnector(const UUID& uuid) const;
    Input* getInput(const UUID& uuid) const;
    Output* getOutput(const UUID& uuid) const;
    Slot* getSlot(const UUID& uuid) const;
    Trigger* getTrigger(const UUID& uuid) const;

    void makeParameterConnectable(csapex::param::ParameterPtr);
    void makeParameterNotConnectable(csapex::param::ParameterPtr);
    InputWeakPtr getParameterInput(const std::string& name) const;
    OutputWeakPtr getParameterOutput(const std::string& name) const;


    void removeInput(const UUID& uuid);
    void removeOutput(const UUID& uuid);
    void removeSlot(const UUID& uuid);
    void removeTrigger(const UUID& uuid);

    std::vector<ConnectablePtr> getAllConnectors() const;
    std::vector<InputPtr> getAllInputs() const;
    std::vector<OutputPtr> getAllOutputs() const;

    std::vector<SlotPtr> getSlots() const;
    std::vector<TriggerPtr> getTriggers() const;

    bool isWaitingForTrigger() const;
    virtual bool canProcess() const override;
    bool canReceive() const;
    bool canSend() const;
    bool areAllInputsAvailable() const;

    void makeParametersConnectable();

    bool isSource() const;
    void setIsSource(bool source);

    bool isSink() const;
    void setIsSink(bool sink);

    int getLevel() const;
    void setLevel(int level);

    std::vector<TimerPtr> extractLatestTimers();

private:
    void updateParameterValue(Connectable* source);

public:
    bool tick();

    virtual void startProcessingMessages() override;
    void finishProcessingMessages(bool was_executed);

    void checkTransitions(bool try_fire = true);

    void checkParameters();    
    void checkIO();

    void enableIO(bool enable);
    void enableInput(bool enable);
    void enableOutput(bool enable);
    void enableSlots(bool enable);
    void enableTriggers(bool enable);

    void triggerError(bool e, const std::string& what);

    void killExecution();

    void sendMessages();
    void notifyMessagesProcessed();

public:
    boost::signals2::signal<void()> panic;

    boost::signals2::signal<void()> ticked;
    boost::signals2::signal<void(bool)> enabled;

    boost::signals2::signal<void(NodeWorker* worker, int type, long stamp)> timerStarted;
    boost::signals2::signal<void(NodeWorker* worker, long stamp)> timerStopped;

    boost::signals2::signal<void(NodeWorker* worker)> startProfiling;
    boost::signals2::signal<void(NodeWorker* worker)> stopProfiling;

    boost::signals2::signal<void()> threadChanged;
    boost::signals2::signal<void(bool)> errorHappened;

    boost::signals2::signal<void()> messages_processed;
    boost::signals2::signal<void()> processRequested;
    boost::signals2::signal<void()> checkTransitionsRequested;

    boost::signals2::signal<void()> parametersChanged;

    boost::signals2::signal<void(std::function<void()>)> executionRequested;

    boost::signals2::signal<void (ConnectablePtr)> connectorCreated;
    boost::signals2::signal<void (ConnectablePtr)> connectorRemoved;

    boost::signals2::signal<void (Connectable*, Connectable*)> connectionInProgress;
    boost::signals2::signal<void (Connectable*)> connectionDone;
    boost::signals2::signal<void (Connectable*)> connectionStart;


    boost::signals2::signal<void()> nodeStateChanged;

private:
    void removeInput(Input *in);
    void removeOutput(Output *out);
    void removeSlot(Slot *out);
    void removeTrigger(Trigger *out);

    void connectConnector(Connectable* c);
    void disconnectConnector(Connectable* c);

    template <typename T>
    void makeParameterConnectableImpl(csapex::param::ParameterPtr);
    void publishParameters();
    void publishParameter(csapex::param::Parameter *p);
    void publishParameterOn(const csapex::param::Parameter &p, Output *out);

    void assertNotInGuiThread();

    void triggerNodeStateChanged();

    void finishTimer(TimerPtr t);

    void updateTransitionConnections();

    void errorEvent(bool error, const std::string &msg, ErrorLevel level);

private:
    NodeStatePtr node_state_;
    NodeModifierPtr modifier_;

    bool is_setup_;
    State state_;

    int next_input_id_;
    int next_output_id_;
    int next_trigger_id_;
    int next_slot_id_;

    Trigger* trigger_tick_done_;
    Trigger* trigger_process_done_;

    std::map<std::string, InputWeakPtr> param_2_input_;
    std::map<std::string, OutputWeakPtr> param_2_output_;

    std::map<Input*,csapex::param::Parameter*> input_2_param_;
    std::map<Output*,csapex::param::Parameter*> output_2_param_;

    std::map<Slot*, boost::signals2::connection> slot_connections_;

    std::map<Trigger*, boost::signals2::connection> trigger_triggered_connections_;
    std::map<Trigger*, boost::signals2::connection> trigger_handled_connections_;


    std::vector<boost::signals2::connection> connections;

    int ticks_;

    bool source_;
    bool sink_;
    int level_;

    mutable std::recursive_mutex sync;
    mutable std::recursive_mutex state_mutex_;

    std::recursive_mutex timer_mutex_;
    std::vector<TimerPtr> timer_history_;

    std::atomic<bool> profiling_;
    TimerPtr current_process_timer_;
};

}

#endif // NODE_WORKER_H
