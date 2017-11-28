#ifndef NODE_HANDLE_H
#define NODE_HANDLE_H

/// PROJECT
#include <csapex/model/connectable_owner.h>
#include <csapex/model/model_fwd.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/param/param_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/utility/utility_fwd.h>
#include <csapex/utility/rate.h>
#include <csapex/utility/slim_signal.hpp>
#include <csapex/model/connector_description.h>
#include <csapex/model/connectable_vector.h>
#include <csapex/serialization/serialization_fwd.h>

/// SYSTEM
#include <vector>
#include <string>
#include <unordered_map>

namespace csapex
{

class CSAPEX_EXPORT NodeHandle : public NodeModifier, public ConnectableOwner, public std::enable_shared_from_this<NodeHandle>
{
public:
    NodeHandle(const std::string& type, const UUID &uuid,
               NodePtr node, UUIDProviderPtr uuid_provider,
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
    bool isParameterInput(const UUID& id) const override;

    Output* addOutput(TokenDataConstPtr type, const std::string& label) override;
    void manageOutput(OutputPtr out);
    bool isParameterOutput(const UUID& id) const override;

    Slot* addSlot(TokenDataConstPtr type, const std::string& label, std::function<void (Slot*,const TokenPtr& )> callback, bool active, bool blocking) override;
    Slot* addSlot(TokenDataConstPtr type, const std::string& label, std::function<void (const TokenPtr& )> callback, bool active, bool blocking) override;
    Slot* addSlot(TokenDataConstPtr type, const std::string& label, std::function<void ()> callback, bool active, bool blocking) override;
    void manageSlot(SlotPtr s);

    Event* addEvent(TokenDataConstPtr type, const std::string& label) override;
    void manageEvent(EventPtr t);

    InputPtr addInternalInput(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label, bool optional);
    OutputPtr addInternalOutput(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label);
    SlotPtr addInternalSlot(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label, std::function<void (const TokenPtr& )> callback);
    EventPtr addInternalEvent(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label);

    ConnectablePtr getConnector(const UUID& uuid) const;
    ConnectablePtr getConnectorNoThrow(const UUID& uuid) const noexcept;
    virtual InputPtr getInput(const UUID& uuid) const noexcept;
    virtual OutputPtr getOutput(const UUID& uuid) const noexcept;
    virtual SlotPtr getSlot(const UUID& uuid) const noexcept;
    virtual EventPtr getEvent(const UUID& uuid) const noexcept;


    void removeInput(const UUID& uuid) override;
    void removeOutput(const UUID& uuid) override;
    void removeSlot(const UUID& uuid) override;
    void removeEvent(const UUID& uuid) override;


    void makeParameterConnectable(csapex::param::ParameterPtr);
    void makeParameterNotConnectable(csapex::param::ParameterPtr);
    InputWeakPtr getParameterInput(const std::string& name) const;
    OutputWeakPtr getParameterOutput(const std::string& name) const;


    std::vector<ConnectorDescription> getExternalInputDescriptions() const;
    std::vector<ConnectorDescription> getInternalInputDescriptions() const;
    std::vector<ConnectorDescription> getExternalOutputDescriptions() const;
    std::vector<ConnectorDescription> getInternalOutputDescriptions() const;
    std::vector<ConnectorDescription> getExternalSlotDescriptions() const;
    std::vector<ConnectorDescription> getInternalSlotDescriptions() const;
    std::vector<ConnectorDescription> getExternalEventDescriptions() const;
    std::vector<ConnectorDescription> getInternalEventDescriptions() const;

    std::vector<InputPtr> getExternalInputs() const override;
    std::vector<InputPtr> getInternalInputs() const;
    std::vector<OutputPtr> getExternalOutputs() const override;
    std::vector<OutputPtr> getInternalOutputs() const;
    std::vector<SlotPtr> getExternalSlots() const override;
    std::vector<SlotPtr> getInternalSlots() const;
    std::vector<EventPtr> getExternalEvents() const override;
    std::vector<EventPtr> getInternalEvents() const;

    std::vector<ConnectablePtr> getExternalConnectors() const override;

    std::map<std::string, InputWeakPtr>& paramToInputMap();
    std::map<std::string, OutputWeakPtr>& paramToOutputMap();

    std::unordered_map<UUID,csapex::param::Parameter*,UUID::Hasher>& inputToParamMap();
    std::unordered_map<UUID,csapex::param::Parameter*,UUID::Hasher>& outputToParamMap();

    bool isSource() const override;
    bool isSink() const override;
    bool hasConnectionsIncoming() const;
    bool hasConnectionsOutgoing() const;

    bool isVariadic() const;
    bool hasVariadicInputs() const;
    bool hasVariadicOutputs() const;
    bool hasVariadicEvents() const;
    bool hasVariadicSlots() const;

    bool isActive() const;
    void setActive(bool active);

    void triggerNodeStateChanged();

    UUIDProvider* getUUIDProvider();
    AUUID getSubgraphAUUID() const;

    bool isGraph() const;

    Rate& getRate();
    const Rate& getRate() const;

public:
    // TODO: get rid of
    void updateParameterValue(Connectable* source);

    void updateLoggerLevel();

public:
    slim_signal::Signal<void ()> stopped;
    slim_signal::Signal<void ()> node_removed;

    slim_signal::Signal<void (ConnectablePtr, bool)> connector_created;
    slim_signal::Signal<void (ConnectablePtr, bool)> connector_removed;

    slim_signal::Signal<void (ConnectablePtr)> connection_added;
    slim_signal::Signal<void (ConnectablePtr)> connection_removed;
    slim_signal::Signal<void (ConnectablePtr)> connection_start;

    slim_signal::Signal<void()> parameters_changed;

    slim_signal::Signal<void()> node_state_changed;

    slim_signal::Signal<void()> activation_changed;

    slim_signal::Signal<void()> might_be_enabled;

    slim_signal::Signal<void(std::function<void()>)> execution_requested;

    slim_signal::ObservableSignal<void(StreamableConstPtr)> raw_data_connection;



    void connectConnector(Connectable* c);
    void disconnectConnector(Connectable* c);

private:
    void removeInput(Input *in);
    void removeOutput(Output *out);
    void removeSlot(Slot *out);
    void removeEvent(Event *out);

    template <typename T>
    void makeParameterConnectableImpl(csapex::param::ParameterPtr);

    template <typename T>
    void makeParameterConnectableTyped(csapex::param::ParameterPtr);

protected:
    mutable std::recursive_mutex sync;

    NodePtr node_;
    NodeStatePtr node_state_;
    NodeRunnerWeakPtr node_runner_;

    ConnectableVector<Input> external_inputs_;
    ConnectableVector<Output> external_outputs_;
    ConnectableVector<Event> external_events_;
    ConnectableVector<Slot> external_slots_;

    ConnectableVector<Input> internal_inputs_;
    ConnectableVector<Output> internal_outputs_;
    ConnectableVector<Event> internal_events_;
    ConnectableVector<Slot> internal_slots_;

    std::string node_type_;

    InputTransitionPtr transition_in_;
    OutputTransitionPtr transition_out_;

    std::map<std::string, InputWeakPtr> param_2_input_;
    std::map<std::string, OutputWeakPtr> param_2_output_;

    std::unordered_map<UUID,csapex::param::Parameter*, UUID::Hasher> input_2_param_;
    std::unordered_map<UUID,csapex::param::Parameter*, UUID::Hasher> output_2_param_;

private:
    UUIDProviderPtr uuid_provider_;

    graph::VertexWeakPtr vertex_;

    Rate rate_;

    std::map<Connectable*, std::vector<slim_signal::Connection>> connections_;

public:
    long guard_;
};

}

#endif // NODE_HANDLE_H

