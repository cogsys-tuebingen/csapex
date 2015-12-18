#ifndef NODE_HANDLE_H
#define NODE_HANDLE_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/model/unique.h>
#include <csapex/param/param_fwd.h>

/// SYSTEM
#include <vector>
#include <string>
#include <boost/signals2/signal.hpp>

namespace csapex
{

class NodeHandle : public Unique
{
public:
    NodeHandle(const std::string& type, const UUID &uuid,
               NodePtr node,
               InputTransitionPtr transition_in, OutputTransitionPtr transition_out);
    virtual ~NodeHandle();    

    std::string getType() const;
    NodeWeakPtr getNode() const;

    InputTransition* getInputTransition() const;
    OutputTransition* getOutputTransition() const;


    void setNodeState(NodeStatePtr memento);
    NodeStatePtr getNodeState();
    NodeStatePtr getNodeStateCopy() const;

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


    void removeInput(const UUID& uuid);
    void removeOutput(const UUID& uuid);
    void removeSlot(const UUID& uuid);
    void removeTrigger(const UUID& uuid);


    void makeParameterConnectable(csapex::param::ParameterPtr);
    void makeParameterNotConnectable(csapex::param::ParameterPtr);
    InputWeakPtr getParameterInput(const std::string& name) const;
    OutputWeakPtr getParameterOutput(const std::string& name) const;


    std::vector<ConnectablePtr> getAllConnectors() const;
    std::vector<InputPtr> getAllInputs() const;
    std::vector<OutputPtr> getAllOutputs() const;

    std::vector<SlotPtr> getSlots() const;
    std::vector<TriggerPtr> getTriggers() const;

    std::map<std::string, InputWeakPtr>& paramToInputMap();
    std::map<std::string, OutputWeakPtr>& paramToOutputMap();

    std::map<Input*,csapex::param::Parameter*>& inputToParamMap();
    std::map<Output*,csapex::param::Parameter*>& outputToParamMap();

public:
    // TODO: get rid of
    void updateParameterValue(Connectable* source);

public:
    boost::signals2::signal<void (ConnectablePtr)> connectorCreated;
    boost::signals2::signal<void (ConnectablePtr)> connectorRemoved;

    boost::signals2::signal<void (Connectable*, Connectable*)> connectionInProgress;
    boost::signals2::signal<void (Connectable*)> connectionDone;
    boost::signals2::signal<void (Connectable*)> connectionStart;

    boost::signals2::signal<void()> parametersChanged;

    boost::signals2::signal<void()> nodeStateChanged;

    boost::signals2::signal<void()> mightBeEnabled;


    // TODO: get rid of
    boost::signals2::signal<void(std::function<void()>)> executionRequested;

protected:
    void connectConnector(Connectable* c);
    void disconnectConnector(Connectable* c);

private:
    void removeInput(Input *in);
    void removeOutput(Output *out);
    void removeSlot(Slot *out);
    void removeTrigger(Trigger *out);


    void triggerNodeStateChanged();


    template <typename T>
    void makeParameterConnectableImpl(csapex::param::ParameterPtr);


protected:
    mutable std::recursive_mutex sync;

    NodePtr node_;
    NodeStatePtr node_state_;

    std::vector<InputPtr> inputs_;
    std::vector<OutputPtr> outputs_;
    std::vector<TriggerPtr> triggers_;
    std::vector<SlotPtr> slots_;

    std::string node_type_;

    InputTransitionPtr transition_in_;
    OutputTransitionPtr transition_out_;

    std::map<std::string, InputWeakPtr> param_2_input_;
    std::map<std::string, OutputWeakPtr> param_2_output_;

    std::map<Input*,csapex::param::Parameter*> input_2_param_;
    std::map<Output*,csapex::param::Parameter*> output_2_param_;

private:
    int next_input_id_;
    int next_output_id_;
    int next_trigger_id_;
    int next_slot_id_;



    std::map<Slot*, boost::signals2::connection> slot_connections_;
    std::map<Trigger*, boost::signals2::connection> trigger_triggered_connections_;
    std::map<Trigger*, boost::signals2::connection> trigger_handled_connections_;

};

}

#endif // NODE_HANDLE_H

