#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/model/connection.h>
#include <csapex/command/meta.h>
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <QObject>
#include <QTimer>
#include <QMenu>
#include <map>
#include <deque>
#include <boost/function.hpp>

namespace csapex {

namespace command {
}


class Graph : public QObject
{
    Q_OBJECT

    friend class GraphIO;
    friend class command::AddNode;
    friend class command::AddConnection;
    friend class command::DeleteConnection;
    friend class command::DeleteNode;
    friend class Overlay;

    friend class Template;
    friend class CommandDispatcher;

public:
    static const std::string namespace_separator;

    typedef boost::shared_ptr<Graph> Ptr;

private:
    Graph();

public:
    virtual ~Graph();

    void init(CommandDispatcher* dispatcher);
    void stop();

    Graph::Ptr findSubGraph(const UUID& uuid);

    Node* findNode(const UUID& uuid);
    Node* findNodeNoThrow(const UUID& uuid);
    Node* findNodeForConnector(const UUID &uuid);


    Connectable* findConnector(const UUID &uuid);


    bool handleConnectionSelection(int id, bool add);
    Command::Ptr deleteConnectionByIdCommand(int id);

    Command::Ptr deleteConnectionFulcrumCommand(int connection, int fulcrum);

    Command::Ptr deleteAllConnectionFulcrumsCommand(int connection);
    Command::Ptr deleteAllConnectionFulcrumsCommand(Connection::Ptr connection);

    Command::Ptr deleteConnectionById(int id);
    Command::Ptr deleteSelectedConnectionsCmd();
    int noSelectedConnections();


    Connection::Ptr getConnectionWithId(int id);
    Connection::Ptr getConnection(Connection::Ptr);
    int getConnectionId(Connection::Ptr);

    std::string makeUUIDPrefix(const std::string& name);

    Command::Ptr clear();

    TemplatePtr toTemplate(const std::string& name) const;

    void fillContextMenuForSelection(QMenu* menu, std::map<QAction *, boost::function<void()> > &handler);


    int countNodes();
    int countSelectedNodes();
    void selectNode(Node* node, bool add = false);
    void deselectNodes();

    Command::Ptr deleteSelectedNodesCmd();
    Command::Ptr groupSelectedNodesCmd();

    void handleNodeSelection(Node* node, bool add);

    void foreachNode(boost::function<void (Node*)> f, boost::function<bool (Node*)> pred);
    void foreachBox(boost::function<void (Box*)> f, boost::function<bool (Box*)> pred);

public Q_SLOTS:
    void reset();
    void tick();
    void clearSelection();
    void selectAll();

    void toggleBoxSelection(Box* box);
    void boxMoved(Box* box, int dx, int dy);

    Command::Ptr moveSelectedBoxes(const QPoint& delta);

    void verify();

Q_SIGNALS:
    void stateChanged();
    void dirtyChanged(bool);
    void selectionChanged();

    void connectionAdded(Connection*);
    void connectionDeleted(Connection*);

    void nodeAdded(NodePtr);
    void nodeRemoved(NodePtr);

    void sig_tick();

private:
    void deselectConnections();
    void deselectConnectionById(int id);
    void selectConnectionById(int id, bool add = false);
    bool isConnectionWithIdSelected(int id);

    TemplatePtr convertSelectionToTemplate(std::vector<std::pair<UUID, UUID> > &connections) const;
    TemplatePtr generateTemplate(TemplatePtr templ, std::vector<std::pair<UUID, UUID> > &connections, bool only_selected) const;


private: /// ONLY COMMANDS / NOT UNDOABLE
    void addNode(NodePtr node);
    void deleteNode(const UUID &uuid);

    bool addConnection(Connection::Ptr connection);
    void deleteConnection(Connection::Ptr connection);

    void verifyAsync();

protected:
    std::vector<NodePtr> nodes_;
    std::vector<Connectable*> connectors_;
    std::vector<Connection::Ptr> visible_connections;

    CommandDispatcher* dispatcher_;

    std::map<std::string, int> uuids;

    QTimer* timer_;
};

}

#endif // GRAPH_H
