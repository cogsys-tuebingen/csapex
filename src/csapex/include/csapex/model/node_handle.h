#ifndef NODE_HANDLE_H
#define NODE_HANDLE_H

/// PROJECT
#include <csapex/model/connectable_owner.h>
#include <csapex/model/model_fwd.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/unique.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/param/param_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/utility/rate.h>
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <vector>
#include <string>

namespace csapex
{

class CSAPEX_EXPORT NodeHandle : public Unique, public NodeModifier, public ConnectableOwner, public std::enable_shared_from_this<NodeHandle>
{
public:
    NodeHandle(const std::string& type, const UUID &uuid,
               NodePtr node, UUIDProvider *uuid_provider,
               InputTransitionPtr transition_in, OutputTransitionPtr transition_out);
    virtual ~NodeHandle();    

    void stop();

    std::string getType() const;
    NodeWeakPtr getNode() const;

    void setVertex(graph::VertexWeakPtr vertex);
    graph::VertexPtr getVertex() const;

    InputTransition* getInputTransition() const;
    OutputTransition* getOutputTransition() const;

    void setNodeState(NodeStatePtr memento);
    NodeStatePtr getNodeState();
    NodeStatePtr getNodeStateCopy() const;

    void setNodeRunner(NodeRunnerWeakPtr runner);
    NodeRunnerPtr getNodeRunner() const;


    Input* addInput(TokenDataConstPtr type, const std::string& label, bool optional) override;
    void manageInput(InputPtr in);
    bool isParameterInput(Input* in) const override;

    Output* addOutput(TokenDataConstPtr type, const std::string& label) override;
    void manageOutput(OutputPtr out);
    bool isParameterOutput(Output* out) const override;

    Slot* addSlot(TokenDataConstPtr type, const std::string& label, std::function<void (Slot*,const TokenPtr& )> callback, bool active, bool asynchronous) override;
    Slot* addSlot(TokenDataConstPtr type, const std::string& label, std::function<void (const TokenPtr& )> callback, bool active, bool asynchronous) override;
    Slot* addSlot(TokenDataConstPtr type, const std::string& label, std::function<void ()> callback, bool active, bool asynchronous) override;
    void manageSlot(SlotPtr s);

    Event* addEvent(TokenDataConstPtr type, const std::string& label) override;
    void manageEvent(EventPtr t);

    InputPtr addInternalInput(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label, bool optional);
    OutputPtr addInternalOutput(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label);
    SlotPtr addInternalSlot(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label, std::function<void (const TokenPtr& )> callback);
    EventPtr addInternalEvent(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label);

    void removeInternalPorts();

    ConnectablePtr getConnector(const UUID& uuid) const;
    ConnectablePtr getConnectorNoThrow(const UUID& uuid) const noexcept;
    virtual InputPtr getInput(const UUID& uuid) const;
    virtual OutputPtr getOutput(const UUID& uuid) const;
    virtual SlotPtr getSlot(const UUID& uuid) const;
    virtual EventPtr getEvent(const UUID& uuid) const;


    void removeInput(const UUID& uuid) override;
    void removeOutput(const UUID& uuid) override;
    void removeSlot(const UUID& uuid) override;
    void removeEvent(const UUID& uuid) override;


    void makeParameterConnectable(csapex::param::ParameterPtr);
    void makeParameterNotConnectable(csapex::param::ParameterPtr);
    InputWeakPtr getParameterInput(const std::string& name) const;
    OutputWeakPtr getParameterOutput(const std::string& name) const;


    std::vector<ConnectablePtr> getExternalConnectors() const override;

    std::vector<InputPtr> getExternalInputs() const override;
    std::vector<InputPtr> getInternalInputs() const;

    std::vector<OutputPtr> getExternalOutputs() const override;
    std::vector<OutputPtr> getInternalOutputs() const;

    std::vector<SlotPtr> getExternalSlots() const override;
    std::vector<SlotPtr> getInternalSlots() const;


    std::vector<EventPtr> getExternalEvents() const override;
    std::vector<EventPtr> getInternalEvents() const;

    std::map<std::string, InputWeakPtr>& paramToInputMap();
    std::map<std::string, OutputWeakPtr>& paramToOutputMap();

    std::map<Input*,csapex::param::Parameter*>& inputToParamMap();
    std::map<Output*,csapex::param::Parameter*>& outputToParamMap();


    bool isSource() const override;
    void setIsSource(bool source) override;

    bool isSink() const override;
    void setIsSink(bool sink) override;

    bool isActive() const;
    void setActive(bool active);

    void triggerNodeStateChanged();

    UUIDProvider* getUUIDProvider();

    bool isGraph() const;

    Rate& getRate();
    const Rate& getRate() const;

public:
    // TODO: get rid of
    void updateParameterValue(Connectable* source);

    void updateLoggerLevel();

public:
    csapex::slim_signal::Signal<void ()> nodeRemoved;

    csapex::slim_signal::Signal<void (ConnectablePtr)> connectorCreated;
    csapex::slim_signal::Signal<void (ConnectablePtr)> connectorRemoved;

    csapex::slim_signal::Signal<void (Connectable*, Connectable*)> connectionInProgress;
    csapex::slim_signal::Signal<void (Connectable*)> connectionDone;
    csapex::slim_signal::Signal<void (Connectable*)> connectionStart;

    csapex::slim_signal::Signal<void()> parametersChanged;

    csapex::slim_signal::Signal<void()> nodeStateChanged;

    csapex::slim_signal::Signal<void()> activationChanged;

    csapex::slim_signal::Signal<void()> mightBeEnabled;

    csapex::slim_signal::Signal<void(std::function<void()>)> executionRequested;

    void connectConnector(Connectable* c);
    void disconnectConnector(Connectable* c);

private:
    void removeInput(Input *in);
    void removeOutput(Output *out);
    void removeSlot(Slot *out);
    void removeEvent(Event *out);

    template <typename T>
    void makeParameterConnectableImpl(csapex::param::ParameterPtr);

protected:
    mutable std::recursive_mutex sync;

    NodePtr node_;
    NodeStatePtr node_state_;
    NodeRunnerWeakPtr node_runner_;

    std::vector<InputPtr> external_inputs_;
    std::vector<OutputPtr> external_outputs_;
    std::vector<EventPtr> external_events_;
    std::vector<SlotPtr> external_slots_;

    std::vector<InputPtr> internal_inputs_;
    std::vector<OutputPtr> internal_outputs_;
    std::vector<EventPtr> internal_events_;
    std::vector<SlotPtr> internal_slots_;

    std::string node_type_;

    InputTransitionPtr transition_in_;
    OutputTransitionPtr transition_out_;

    std::map<std::string, InputWeakPtr> param_2_input_;
    std::map<std::string, OutputWeakPtr> param_2_output_;

    std::map<Input*,csapex::param::Parameter*> input_2_param_;
    std::map<Output*,csapex::param::Parameter*> output_2_param_;

private:
    UUIDProvider* uuid_provider_;

    graph::VertexWeakPtr vertex_;

    Rate rate_;

    std::map<Connectable*, std::vector<csapex::slim_signal::Connection>> connections_;

    bool source_;
    bool sink_;
};

}

#endif // NODE_HANDLE_H

