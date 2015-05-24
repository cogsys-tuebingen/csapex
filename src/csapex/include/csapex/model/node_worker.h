#ifndef NODE_WORKER_H
#define NODE_WORKER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_param/parameter.h>
#include <csapex/model/unique.h>
#include <csapex/model/error_state.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <QObject>
#include <QTimer>
#include <QThread>
#include <map>
#include <functional>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <atomic>
#include <boost/signals2/signal.hpp>

namespace csapex {

class NodeWorker : public QObject, public ErrorState, public Unique
{
    Q_OBJECT

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
        PROCESSING,
        WAITING_FOR_OUTPUTS,
        WAITING_FOR_RESET
    };

public:
    NodeWorker(const std::string& type, const UUID& uuid, Settings& settings, NodePtr node);
    ~NodeWorker();

    InputTransition* getInputTransition() const;
    OutputTransition* getOutputTransition() const;

    void setNodeState(NodeStatePtr memento);
    NodeStatePtr getNodeState();
    NodeStatePtr getNodeStateCopy() const;


    void stop();
    void waitUntilFinished();
    void reset();

    void triggerCheckTransitions();
    void triggerProcess();
    void triggerSwitchThreadRequest(QThread* thread, int id);
    void triggerPanic();

    Node* getNode() const;
    State getState() const;

    std::string getType() const;

    bool isEnabled() const;
    void setEnabled(bool e);

    bool isPaused() const;

    void setProfiling(bool profiling);
    bool isProfiling() const;

    void setIOError(bool error);

    /* REMOVE => UI*/ void setMinimized(bool min);

    Input* addInput(ConnectionTypePtr type, const std::string& label, bool dynamic, bool optional);
    Output* addOutput(ConnectionTypePtr type, const std::string& label, bool dynamic);
    Slot* addSlot(const std::string& label, std::function<void ()> callback, bool active);
    Trigger* addTrigger(const std::string& label);

    Connectable* getConnector(const UUID& uuid) const;
    Input* getInput(const UUID& uuid) const;
    Output* getOutput(const UUID& uuid) const;
    Slot* getSlot(const UUID& uuid) const;
    Trigger* getTrigger(const UUID& uuid) const;

    void makeParameterConnectable(param::Parameter*);
    void makeParameterNotConnectable(param::ParameterPtr);
    Input* getParameterInput(const std::string& name) const;
    Output* getParameterOutput(const std::string& name) const;

    /* NAMING */ void registerInput(InputPtr in);
    /* NAMING */ void registerOutput(OutputPtr out);
    /* NAMING */ void registerSlot(SlotPtr s);
    /* NAMING */ void registerTrigger(TriggerPtr t);

    void removeInput(const UUID& uuid);
    void removeOutput(const UUID& uuid);
    void removeSlot(const UUID& uuid);
    void removeTrigger(const UUID& uuid);

    std::vector<Connectable*> getAllConnectors() const;
    std::vector<Input*> getAllInputs() const;
    std::vector<Output*> getAllOutputs() const;

    std::vector<Input*> getMessageInputs() const;
    std::vector<Output*> getMessageOutputs() const;
    std::vector<Slot*> getSlots() const;
    std::vector<Trigger*> getTriggers() const;

    std::vector<Input*> getParameterInputs() const;
    std::vector<Output*> getParameterOutputs() const;

    bool canProcess();
    bool canReceive();
    bool areAllInputsAvailable();

    void makeParametersConnectable();

    bool isTickEnabled() const;
    void setTickEnabled(bool enabled);

    bool isSource() const;
    void setIsSource(bool source);

    bool isSink() const;
    void setIsSink(bool sink);

    int getLevel() const;
    void setLevel(int level);

    std::vector<TimerPtr> extractLatestTimers();

private:
    void updateParameterValue(Connectable* source);

public Q_SLOTS:
    void processMessages();

    void prepareForNextProcess();
    void checkTransitions();


    void checkParameters();    
    void checkIO();

    void enableIO(bool enable);
    void enableInput(bool enable);
    void enableOutput(bool enable);
    void enableSlots(bool enable);
    void enableTriggers(bool enable);

    void setTickFrequency(double f);
    void tick();

    void triggerError(bool e, const std::string& what);

    void pause(bool pause);
    void killExecution();

//    bool canSendMessages();
    void sendMessages();
    void notifyMessagesProcessed();

    /** SLOTS **/
    void outputConnectionChanged(Connectable*);

public:
    boost::signals2::signal<void()> panic;
    boost::signals2::signal<void()> messages_processed;

    boost::signals2::signal<void()> ticked;
    boost::signals2::signal<void(bool)> enabled;

    boost::signals2::signal<void(NodeWorker* worker, int type, long stamp)> timerStarted;
    boost::signals2::signal<void(NodeWorker* worker, long stamp)> timerStopped;

    boost::signals2::signal<void(NodeWorker* worker)> startProfiling;
    boost::signals2::signal<void(NodeWorker* worker)> stopProfiling;

    boost::signals2::signal<void()> threadChanged;
    boost::signals2::signal<void(bool)> errorHappened;

Q_SIGNALS:
    void messagesProcessed();

    void connectionInProgress(Connectable*, Connectable*);
    void connectionDone(Connectable*);
    void connectionStart(Connectable*);

    void connectorCreated(Connectable*);
    void connectorRemoved(Connectable*);

    void nodeStateChanged();

    void threadSwitchRequested(QThread*, int);
    void tickRequested();

    void processRequested();
    void checkTransitionsRequested();

private Q_SLOTS:
    void switchThread(QThread* thread, int id);

private:
    void removeInput(Input *in);
    void removeOutput(Output *out);
    void removeSlot(Slot *out);
    void removeTrigger(Trigger *out);

    void connectConnector(Connectable* c);
    void disconnectConnector(Connectable* c);

    template <typename T>
    void makeParameterConnectableImpl(param::Parameter*);
    void publishParameters();
    void publishParameter(param::Parameter *p);

    void assertNotInGuiThread();

    void triggerNodeStateChanged();

    void finishTimer(TimerPtr t);

    void setState(State state);

    void updateTransitions();

    void startRunning();
    void stopRunning();

    void errorEvent(bool error, const std::string &msg, ErrorLevel level);

private:
    Settings& settings_;

    std::string node_type_;
    NodePtr node_;    
    NodeStatePtr node_state_;
    NodeModifierPtr modifier_;

    InputTransitionPtr transition_in_;
    OutputTransitionPtr transition_out_;

    bool is_setup_;
    State state_;

    std::vector<InputPtr> inputs_;
    std::vector<OutputPtr> outputs_;
    std::vector<TriggerPtr> triggers_;
    std::vector<SlotPtr> slots_;

    Trigger* trigger_tick_done_;
    Trigger* trigger_process_done_;

    std::vector<InputPtr> parameter_inputs_;
    std::vector<OutputPtr> parameter_outputs_;

    std::map<std::string, InputPtr> param_2_input_;
    std::map<std::string, OutputPtr> param_2_output_;

    std::map<Input*,param::Parameter*> input_2_param_;
    std::map<Output*,param::Parameter*> output_2_param_;

    std::vector<boost::signals2::connection> connections;
    std::vector<QObject*> callbacks;

    QTimer* tick_timer_;
    bool tick_enabled_;
    bool tick_immediate_;
    int ticks_;

    bool source_;
    bool sink_;
    int level_;

    mutable std::recursive_mutex sync;
    mutable std::recursive_mutex state_mutex_;

    std::recursive_mutex timer_mutex_;
    std::vector<TimerPtr> timer_history_;

    bool thread_initialized_;
    bool paused_;
    bool stop_;
    mutable std::recursive_mutex pause_mutex_;
    std::condition_variable_any continue_;

    mutable std::recursive_mutex running_mutex_;
    bool running_;
    std::condition_variable_any running_changed_;

    std::atomic<bool> profiling_;
};

}

#endif // NODE_WORKER_H
