#ifndef NODE_HANDLE_H
#define NODE_HANDLE_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/model/unique.h>
#include <csapex/param/param_fwd.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <vector>
#include <string>
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{

class NodeHandle : public Unique, public NodeModifier
{
public:
    NodeHandle(const std::string& type, const UUID &uuid,
               NodePtr node, UUIDProvider *uuid_provider,
               InputTransitionPtr transition_in, OutputTransitionPtr transition_out);
    virtual ~NodeHandle();    

    void stop();

    std::string getType() const;
    NodeWeakPtr getNode() const;

    InputTransition* getInputTransition() const;
    OutputTransition* getOutputTransition() const;

    void setNodeState(NodeStatePtr memento);
    NodeStatePtr getNodeState();
    NodeStatePtr getNodeStateCopy() const;

    int getLevel() const;
    void setLevel(int level);


    Input* addInput(ConnectionTypeConstPtr type, const std::string& label, bool dynamic, bool optional) override;
    void addInput(InputPtr in);
    bool isParameterInput(Input* in) const override;

    Output* addOutput(ConnectionTypeConstPtr type, const std::string& label, bool dynamic) override;
    void addOutput(OutputPtr out);
    bool isParameterOutput(Output* out) const;

    Slot* addSlot(const std::string& label, std::function<void ()> callback, bool active) override;
    void addSlot(SlotPtr s);

    Trigger* addTrigger(const std::string& label) override;
    void addTrigger(TriggerPtr t);


    Connectable* getConnector(const UUID& uuid) const;
    Input* getInput(const UUID& uuid) const;
    Output* getOutput(const UUID& uuid) const;
    Slot* getSlot(const UUID& uuid) const;
    Trigger* getTrigger(const UUID& uuid) const;


    void removeInput(const UUID& uuid) override;
    void removeOutput(const UUID& uuid) override;
    void removeSlot(const UUID& uuid) override;
    void removeTrigger(const UUID& uuid) override;


    void makeParameterConnectable(csapex::param::ParameterPtr);
    void makeParameterNotConnectable(csapex::param::ParameterPtr);
    InputWeakPtr getParameterInput(const std::string& name) const;
    OutputWeakPtr getParameterOutput(const std::string& name) const;


    std::vector<ConnectablePtr> getAllConnectors() const override;
    std::vector<InputPtr> getAllInputs() const override;
    std::vector<OutputPtr> getAllOutputs() const override;

    std::vector<SlotPtr> getAllSlots() const override;
    std::vector<TriggerPtr> getAllTriggers() const override;

    std::map<std::string, InputWeakPtr>& paramToInputMap();
    std::map<std::string, OutputWeakPtr>& paramToOutputMap();

    std::map<Input*,csapex::param::Parameter*>& inputToParamMap();
    std::map<Output*,csapex::param::Parameter*>& outputToParamMap();


    bool isSource() const override;
    void setIsSource(bool source) override;

    bool isSink() const override;
    void setIsSink(bool sink) override;

    void triggerNodeStateChanged();

    UUIDProvider* getUUIDProvider();

public:
    // TODO: get rid of
    void updateParameterValue(Connectable* source);

public:
    csapex::slim_signal::Signal<void ()> nodeRemoved;

    csapex::slim_signal::Signal<void (ConnectablePtr)> connectorCreated;
    csapex::slim_signal::Signal<void (ConnectablePtr)> connectorRemoved;

    csapex::slim_signal::Signal<void (Connectable*, Connectable*)> connectionInProgress;
    csapex::slim_signal::Signal<void (Connectable*)> connectionDone;
    csapex::slim_signal::Signal<void (Connectable*)> connectionStart;

    csapex::slim_signal::Signal<void()> parametersChanged;

    csapex::slim_signal::Signal<void()> nodeStateChanged;

    csapex::slim_signal::Signal<void()> mightBeEnabled;

    csapex::slim_signal::Signal<void(std::function<void()>)> executionRequested;

    void connectConnector(Connectable* c);
    void disconnectConnector(Connectable* c);

private:
    void removeInput(Input *in);
    void removeOutput(Output *out);
    void removeSlot(Slot *out);
    void removeTrigger(Trigger *out);




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
    UUIDProvider* uuid_provider_;

    std::map<Connectable*, std::vector<csapex::slim_signal::Connection>> connections_;

    std::map<Slot*, csapex::slim_signal::Connection> slot_connections_;
    std::map<Trigger*, csapex::slim_signal::Connection> trigger_triggered_connections_;
    std::map<Trigger*, csapex::slim_signal::Connection> trigger_handled_connections_;

    int level_;

    bool source_;
    bool sink_;
};

}

#endif // NODE_HANDLE_H

