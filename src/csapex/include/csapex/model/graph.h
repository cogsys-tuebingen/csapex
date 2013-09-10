#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/model/connection.h>
#include <csapex/command/meta.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QObject>
#include <QTimer>
#include <map>

namespace csapex {

namespace command {
}


class Graph : public QObject
{
    Q_OBJECT

    friend class GraphIO;
    friend class command::AddBox;
    friend class command::AddConnection;
    friend class command::DeleteConnection;
    friend class command::DeleteBox;
    friend class Overlay;
    friend class BoxMeta;

public:
    static const std::string namespace_separator;

    typedef boost::shared_ptr<Graph> Ptr;

public:
    static Graph::Ptr root();
    static void setRoot(Graph::Ptr root);

private:
    static Graph::Ptr root_;

public:
    Graph();
    virtual ~Graph();

    bool isDirty();

    void stop();
    bool canUndo();
    bool canRedo();

    Graph::Ptr findSubGraph(const std::string& uuid);
    BoxPtr findBox(const std::string& uuid, const std::string &ns = "");
    BoxPtr findConnectorOwner(const std::string &uuid, const std::string &ns = "");
    Connector* findConnector(const std::string &uuid, const std::string &ns = "");

    bool handleConnectionSelection(int id, bool add);
    Command::Ptr deleteConnectionByIdCommand(int id);

    Command::Ptr deleteConnectionFulcrumCommand(int connection, int fulcrum);

    Command::Ptr deleteAllConnectionFulcrumsCommand(int connection);
    Command::Ptr deleteAllConnectionFulcrumsCommand(Connection::Ptr connection);

    void deleteConnectionById(int id);
    void deleteSelectedConnections();
    int noSelectedConnections();

    void handleBoxSelection(Box* box, bool add);
    void deleteSelectedBoxes();
    void groupSelectedBoxes();
    void selectBox(Box* box, bool add = false);
    void deselectBoxes();
    int noSelectedBoxes();

    Connection::Ptr getConnectionWithId(int id);
    Connection::Ptr getConnection(Connection::Ptr);
    int getConnectionId(Connection::Ptr);

    std::string makeUUID(const std::string& name);

public Q_SLOTS:
    void undo();
    void redo();
    void clear();
    void reset();
    void tick();
    void clearSelection();
    void selectAll();

    void toggleBoxSelection(Box* box);
    void boxMoved(Box* box, int dx, int dy);

    void moveSelectedBoxes(const QPoint& delta);

Q_SIGNALS:
    void stateChanged();
    void boxAdded(Box*);
    void boxDeleted(Box*);
    void connectionAdded(Connection*);
    void connectionDeleted(Connection*);

private:
    void deselectConnections();
    void deselectConnectionById(int id);
    void selectConnectionById(int id, bool add = false);
    bool isConnectionWithIdSelected(int id);

    TemplatePtr convertSelectionToTemplate(Command::Ptr &pre, Command::Ptr &post, const std::string &group_uuid);

private: /// ONLY COMMANDS / NOT UNDOABLE
    void addBox(BoxPtr box);
    void deleteBox(const std::string &uuid);

    bool addConnection(Connection::Ptr connection);
    void deleteConnection(Connection::Ptr connection);

protected:
    std::vector<BoxPtr> boxes_;
    std::vector<Connector*> connectors_;
    std::vector<Connection::Ptr> visible_connections;


    std::map<std::string, int> uuids;

    QTimer* timer_;
};

}

#endif // GRAPH_H
