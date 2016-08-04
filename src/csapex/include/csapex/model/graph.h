#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/utility/uuid.h>
#include <csapex/model/generator_node.h>
#include <csapex/model/model_fwd.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/variadic_io.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>
#include <map>
#include <unordered_map>
#include <functional>

namespace csapex {

struct RelayMapping
{
    UUID external;
    UUID internal;
};


class Graph : public GeneratorNode, public UUIDProvider, public Variadic
{
    friend class GraphIO;
    friend class GraphFacade;

public:
    typedef std::shared_ptr<Graph> Ptr;

    struct NodeNotFoundException : public std::logic_error
    {
        NodeNotFoundException(const std::string& name)
            : std::logic_error("node " + name + " cannot be found")
        {}
    };

    struct NodeHandleNotFoundException : public std::logic_error
    {
        NodeHandleNotFoundException(const std::string& name)
            : std::logic_error("node handle for node " + name + " cannot be found")
        {}
    };

    typedef std::vector<NodeHandlePtr>::iterator node_iterator;
    typedef std::vector<NodeHandlePtr>::const_iterator node_const_iterator;

public:
    Graph();
    virtual ~Graph();

    virtual void initialize(csapex::NodeHandle* node_handle, const UUID &uuid) override;
    virtual void reset() override;
    void resetActivity();

    virtual void stateChanged() override;

    virtual void activation() override;
    virtual void deactivation() override;

    void clear();

    Node* findNode(const UUID& uuid) const;
    Node* findNodeNoThrow(const UUID& uuid) const noexcept;
    Node* findNodeForConnector(const UUID &uuid) const;

    NodeHandle* findNodeHandle(const UUID& uuid) const;
    NodeHandle* findNodeHandleNoThrow(const UUID& uuid) const noexcept;
    NodeHandle* findNodeHandleForConnector(const UUID &uuid) const;
    NodeHandle* findNodeHandleForConnectorNoThrow(const UUID &uuid) const noexcept;
    NodeHandle* findNodeHandleWithLabel(const std::string& label) const;

    std::vector<NodeHandle*> getAllNodeHandles();

    int getComponent(const UUID& node_uuid) const;

    Connectable* findConnector(const UUID &uuid);
    Connectable* findConnectorNoThrow(const UUID &uuid) noexcept;

    template <typename T>
    T *findConnector(const UUID &uuid)
    {
        return dynamic_cast<T*>(findConnector(uuid));
    }
    template <typename T>
    T *findConnectorNoThrow(const UUID &uuid) noexcept
    {
        return dynamic_cast<T*>(findConnectorNoThrow(uuid));
    }

    ConnectionPtr getConnectionWithId(int id);
    ConnectionPtr getConnection(const UUID& from, const UUID& to);
    ConnectionPtr getConnection(Connectable* from, Connectable* to);
    int getConnectionId(ConnectionPtr);

    std::vector<ConnectionPtr> getConnections();

    int countNodes();

    void addNode(NodeHandlePtr node);
    void deleteNode(const UUID &uuid);

    bool addConnection(ConnectionPtr connection, bool quiet = false);
    void deleteConnection(ConnectionPtr connection);

    void triggerConnectionsAdded();

    // iterators
    node_iterator beginNodes();
    const node_const_iterator beginNodes() const;

    node_iterator endNodes();
    const node_const_iterator endNodes() const;

    // Node interface
    virtual void setup(csapex::NodeModifier& modifier) override;
    virtual void setupParameters(Parameterizable& params) override;
    virtual void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& params,
                         std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)> continuation) override;

    virtual bool isAsynchronous() const override;

    InputPtr createInternalInput(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label, bool dynamic, bool optional);
    OutputPtr createInternalOutput(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label, bool dynamic);
    EventPtr createInternalEvent(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label);
    SlotPtr createInternalSlot(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label, std::function<void (const TokenPtr &)> callback);

    virtual Input* createVariadicInput(TokenDataConstPtr type, const std::string& label, bool optional) override;
    virtual Output* createVariadicOutput(TokenDataConstPtr type, const std::string& label) override;
    virtual Event* createVariadicEvent(TokenDataConstPtr type, const std::string& label) override;
    virtual Slot* createVariadicSlot(TokenDataConstPtr type, const std::string& label, std::function<void(const TokenPtr&)> callback) override;

    virtual void removeVariadicInput(InputPtr input) override;
    virtual void removeVariadicOutput(OutputPtr input) override;
    virtual void removeVariadicEvent(EventPtr input) override;
    virtual void removeVariadicSlot(SlotPtr input) override;

    void removeInternalPorts();

    RelayMapping addForwardingInput(const TokenDataConstPtr& type, const std::string& label, bool optional);
    RelayMapping addForwardingOutput(const TokenDataConstPtr& type, const std::string& label);
    RelayMapping addForwardingSlot(const TokenDataConstPtr& type, const std::string& label);
    RelayMapping addForwardingEvent(const TokenDataConstPtr& type, const std::string& label);

    InputPtr getForwardedInputInternal(const UUID& internal_uuid) const;
    OutputPtr getForwardedOutputInternal(const UUID& internal_uuid) const;
    SlotPtr getForwardedSlotInternal(const UUID& internal_uuid) const;
    EventPtr getForwardedEventInternal(const UUID& internal_uuid) const;

    OutputPtr getRelayForInput(const UUID& external_uuid) const;
    InputPtr getRelayForOutput(const UUID& external_uuid) const;
    EventPtr getRelayForSlot(const UUID& external_uuid) const;
    SlotPtr getRelayForEvent(const UUID& external_uuid) const;

    UUID getForwardedInputExternal(const UUID& internal_uuid) const;
    UUID getForwardedOutputExternal(const UUID& internal_uuid) const;
    UUID getForwardedSlotExternal(const UUID& internal_uuid) const;
    UUID getForwardedEventExternal(const UUID& internal_uuid) const;

    std::vector<UUID> getInternalOutputs() const;
    std::vector<UUID> getInternalInputs() const;
    std::vector<UUID> getInternalSlots() const;
    std::vector<UUID> getInternalEvents() const;

    std::string makeStatusString() const;

    void buildConnectedComponents();

private:
    UUID addForwardingInput(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label, bool optional);
    UUID  addForwardingOutput(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label);
    UUID  addForwardingSlot(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label);
    UUID  addForwardingEvent(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label);

    virtual void notifyMessagesProcessed() override;
    void inputActivation();
    void outputActivation();

public:
    csapex::slim_signal::Signal<void()> state_changed;
    csapex::slim_signal::Signal<void(Graph*)> structureChanged;

    csapex::slim_signal::Signal<void(Connection*)> connectionAdded;
    csapex::slim_signal::Signal<void(Connection*)> connectionDeleted;

    csapex::slim_signal::Signal<void(NodeHandlePtr)> nodeAdded;
    csapex::slim_signal::Signal<void(NodeHandlePtr)> nodeRemoved;

    csapex::slim_signal::Signal<void(ConnectablePtr)> forwardingAdded;
    csapex::slim_signal::Signal<void(ConnectablePtr)> forwardingRemoved;

    csapex::slim_signal::Signal<void(Connectable*,Connectable*)> internalConnectionInProgress;

protected:
    std::vector<NodeHandlePtr> nodes_;
    std::map<NodeHandle*, int> node_component_;

    std::map<NodeHandle*, std::vector<NodeHandle*> > node_parents_;
    std::map<NodeHandle*, std::vector<NodeHandle*> > node_children_;

    std::vector<ConnectionPtr> connections_;

    std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)> continuation_;

    InputTransitionPtr transition_relay_in_;
    OutputTransitionPtr transition_relay_out_;
    std::unordered_map<UUID, SlotPtr, UUID::Hasher> internal_slots_;
    std::unordered_map<UUID, EventPtr, UUID::Hasher> internal_events_;

    std::unordered_map<UUID, OutputPtr, UUID::Hasher> external_to_internal_outputs_;
    std::unordered_map<UUID, InputPtr, UUID::Hasher> external_to_internal_inputs_;
    std::unordered_map<UUID, SlotPtr, UUID::Hasher> external_to_internal_slots_;
    std::unordered_map<UUID, EventPtr, UUID::Hasher> external_to_internal_events_;

    std::unordered_map<UUID, UUID, UUID::Hasher> relay_to_external_output_;
    std::unordered_map<UUID, UUID, UUID::Hasher> relay_to_external_input_;
    std::unordered_map<UUID, UUID, UUID::Hasher> relay_to_external_slot_;
    std::unordered_map<UUID, UUID, UUID::Hasher> relay_to_external_event_;

    bool is_initialized_;

    EventPtr activation_event_;
    EventPtr deactivation_event_;
};

}

#endif // GRAPH_H
