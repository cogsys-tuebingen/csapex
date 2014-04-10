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
    friend class WidgetController;
    friend class Template;
    friend class CommandDispatcher;

public:
    typedef boost::shared_ptr<Graph> Ptr;

public:
    Graph(Settings &settings);

    Settings& getSettings() const;

    virtual ~Graph();

    void init(CommandDispatcher* dispatcher);
    void stop();
    void setPause(bool pause);

    Graph::Ptr findSubGraph(const UUID& uuid);

    Node* findNode(const UUID& uuid);
    Node* findNodeNoThrow(const UUID& uuid);
    Node* findNodeForConnector(const UUID &uuid);


    Connectable* findConnector(const UUID &uuid);


    Command::Ptr deleteConnectionByIdCommand(int id);

    Command::Ptr deleteConnectionFulcrumCommand(int connection, int fulcrum);

    Command::Ptr deleteAllConnectionFulcrumsCommand(int connection);
    Command::Ptr deleteAllConnectionFulcrumsCommand(Connection::Ptr connection);

    Command::Ptr deleteConnectionById(int id);

    Connection::Ptr getConnectionWithId(int id);
    Connection::Ptr getConnection(Connection::Ptr);
    int getConnectionId(Connection::Ptr);

    std::string makeUUIDPrefix(const std::string& name);

    Command::Ptr clear();



    int countNodes();

    void foreachNode(boost::function<void (Node*)> f, boost::function<bool (Node*)> pred);
    void foreachBox(boost::function<void (Box*)> f, boost::function<bool (Box*)> pred);

public Q_SLOTS:
    void reset();
    void tick();

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

private: /// ONLY COMMANDS / NOT UNDOABLE
    void addNode(NodePtr node);
    void deleteNode(const UUID &uuid);

    bool addConnection(Connection::Ptr connection);
    void deleteConnection(Connection::Ptr connection);

    void verifyAsync();

    ///
    /// VIEW DEPENDENT STUFF
    ///
public:
    /*view*/ bool handleConnectionSelection(int id, bool add);
    /*view*/ Command::Ptr deleteSelectedConnectionsCmd();
    /*view*/ void fillContextMenuForSelection(QMenu* menu, std::map<QAction *, boost::function<void()> > &handler);
    /*view*/ int countSelectedNodes();
    /*view*/ void selectNode(Node* node, bool add = false);
    /*view*/ void deselectNodes();

    /*view*/ Command::Ptr deleteSelectedNodesCmd();

    /*view*/ int countSelectedConnections();

    /*view*/ void handleNodeSelection(Node* node, bool add);

public Q_SLOTS:
    /*view*/ void clearSelection();
    /*view*/ void selectAll();

    /*view*/ void toggleBoxSelection(Box* box);
    /*view*/ void boxMoved(Box* box, int dx, int dy);

private:
    /*view*/ void selectConnectionById(int id, bool add = false);
    /*view*/ bool isConnectionWithIdSelected(int id);

protected:
    Settings& settings_;

    std::vector<NodePtr> nodes_;
    std::vector<Connectable*> connectors_;
    std::vector<Connection::Ptr> connections_;

    CommandDispatcher* dispatcher_;

    std::map<std::string, int> uuids_;

    QTimer* timer_;
};

}

#endif // GRAPH_H
