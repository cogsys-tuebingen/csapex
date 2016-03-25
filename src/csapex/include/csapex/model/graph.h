#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/utility/uuid.h>
#include <csapex/model/node.h>
#include <csapex/model/model_fwd.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>
#include <map>
#include <unordered_map>
#include <functional>

namespace csapex {

class Graph : public Node, public UUIDProvider
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

    virtual void stateChanged() override;

    void clear();

    Node* findNode(const UUID& uuid) const;
    Node* findNodeNoThrow(const UUID& uuid) const noexcept;
    Node* findNodeForConnector(const UUID &uuid) const;

    NodeHandle* findNodeHandle(const UUID& uuid) const;
    NodeHandle* findNodeHandleNoThrow(const UUID& uuid) const noexcept;
    NodeHandle* findNodeHandleForConnector(const UUID &uuid) const;
    NodeHandle* findNodeHandleForConnectorNoThrow(const UUID &uuid) const noexcept;

    std::vector<NodeHandle*> getAllNodeHandles();

    int getComponent(const UUID& node_uuid) const;
    int getLevel(const UUID& node_uuid) const;

    Connectable* findConnector(const UUID &uuid);

    template <typename T>
    T *findConnector(const UUID &uuid) {
        return dynamic_cast<T*>(findConnector(uuid));
    }

    ConnectionPtr getConnectionWithId(int id);
    ConnectionPtr getConnection(const UUID& from, const UUID& to);
    ConnectionPtr getConnection(Connectable* from, Connectable* to);
    int getConnectionId(ConnectionPtr);

    std::vector<ConnectionPtr> getConnections();

    int countNodes();

    void addNode(NodeHandlePtr node);
    void deleteNode(const UUID &uuid);

    bool addConnection(ConnectionPtr connection);
    void deleteConnection(ConnectionPtr connection);

    // iterators
    node_iterator beginNodes();
    const node_const_iterator beginNodes() const;

    node_iterator endNodes();
    const node_const_iterator endNodes() const;

    // Node interface
    virtual void setup(csapex::NodeModifier& modifier) override;
    virtual void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& params,
                         std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)> continuation) override;

    virtual bool isAsynchronous() const override;

    std::pair<UUID, UUID> addForwardingInput(const ConnectionTypeConstPtr& type, const std::string& label, bool optional);
    std::pair<UUID, UUID> addForwardingOutput(const ConnectionTypeConstPtr& type, const std::string& label);

    InputPtr getForwardedInput(const UUID& internal_uuid) const;
    OutputPtr getForwardedOutput(const UUID& internal_uuid) const;

    std::vector<UUID> getRelayOutputs() const;
    std::vector<UUID> getRelayInputs() const;

private:
    std::pair<UUID, UUID> addForwardingInput(const UUID& internal_uuid, const UUID &external_uuid, const ConnectionTypeConstPtr& type,
                                             const std::string& label, bool optional);
    std::pair<UUID, UUID> addForwardingOutput(const UUID& internal_uuid, const UUID& external_uuid, const ConnectionTypeConstPtr& type,
                                              const std::string& label);

   /*rename*/ void verify();
    void buildConnectedComponents();
    void assignLevels();

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

protected:
    std::vector<NodeHandlePtr> nodes_;
    std::map<NodeHandle*, int> node_component_;
    std::map<NodeHandle*, int> node_level_;

    std::map<NodeHandle*, std::vector<NodeHandle*> > node_parents_;
    std::map<NodeHandle*, std::vector<NodeHandle*> > node_children_;

    std::vector<ConnectionPtr> connections_;

    std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)> continuation_;

    InputTransitionPtr transition_relay_in_;
    OutputTransitionPtr transition_relay_out_;

    std::map<Input*, OutputPtr> forward_inputs_;
    std::map<Output*, InputPtr> forward_outputs_;

    std::unordered_map<UUID, UUID, UUID::Hasher> relay_to_external_output_;
    std::unordered_map<UUID, UUID, UUID::Hasher> relay_to_external_input_;

    bool is_initialized_;
};

}

#endif // GRAPH_H
