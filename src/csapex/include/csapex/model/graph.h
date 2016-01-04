#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/utility/uuid.h>
#include <csapex/model/node.h>
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>
#include <map>
#include <functional>

namespace csapex {

class Graph : public Node
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

    Connectable *findConnector(const UUID &uuid);

    ConnectionPtr getConnectionWithId(int id);
    ConnectionPtr getConnection(const UUID& from, const UUID& to);
    ConnectionPtr getConnection(Connectable* from, Connectable* to);
    int getConnectionId(ConnectionPtr);

    std::vector<ConnectionPtr> getConnections();

    std::string makeUUIDPrefix(const std::string& name);

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
    virtual void process(csapex::Parameterizable& params,
                         std::function<void (std::function<void ()>)> continuation) override;

    virtual bool isAsynchronous() const override;

    Input* passOutInput(Input* internal);
    Output* passOutOutput(Output* internal);

private:
   /*rename*/ void verify();
    void buildConnectedComponents();
    void assignLevels();

public:
    csapex::slim_signal::Signal<void()> stateChanged;
    csapex::slim_signal::Signal<void(Graph*)> structureChanged;

    csapex::slim_signal::Signal<void(Connection*)> connectionAdded;
    csapex::slim_signal::Signal<void(Connection*)> connectionDeleted;

    csapex::slim_signal::Signal<void(NodeHandlePtr)> nodeAdded;
    csapex::slim_signal::Signal<void(NodeHandlePtr)> nodeRemoved;

protected:
    std::vector<NodeHandlePtr> nodes_;
    std::map<NodeHandle*, int> node_component_;
    std::map<NodeHandle*, int> node_level_;

    std::map<NodeHandle*, std::vector<NodeHandle*> > node_parents_;
    std::map<NodeHandle*, std::vector<NodeHandle*> > node_children_;

    std::vector<ConnectionPtr> connections_;

    std::map<std::string, int> uuids_;

    std::function<void (std::function<void ()>)> continuation_;

    std::map<Input*, OutputPtr> pass_on_inputs_;
    std::map<Output*, Output*> pass_on_outputs_;
    std::map<Output*, bool> received_;
};

}

#endif // GRAPH_H
