#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/connection.h>
#include <csapex/command_meta.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QObject>
#include <QTimer>

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
    void deleteConnectionById(int id);
    void deleteSelectedConnections();
    int noSelectedConnections();

    void handleBoxSelection(Box* box, bool add);
    void deleteSelectedBoxes();
    void groupSelectedBoxes();
    void selectBox(Box* box, bool add = false);
    void deselectBoxes();
    int noSelectedBoxes();

public Q_SLOTS:
    void undo();
    void redo();
    void clear();
    void tick();
    void clearSelection();
    void selectAll();

    void toggleBoxSelection(Box* box);
    void boxMoved(Box* box, int dx, int dy);
    void moveSelectionToBox(Box* box);

Q_SIGNALS:
    void stateChanged();
    void boxAdded(Box*);
    void boxDeleted(Box*);

private:
    Connection::Ptr getConnectionWithId(int id);
    void deselectConnections();
    void deselectConnectionById(int id);
    void selectConnectionById(int id, bool add = false);
    bool isConnectionWithIdSelected(int id);

    command::Meta::Ptr moveSelectionToBoxCommands(const std::string& box_uuid);

private: /// ONLY COMMANDS / NOT UNDOABLE
    void addBox(BoxPtr box);
    void deleteBox(const std::string &uuid);

    bool addConnection(Connection::Ptr connection);
    void deleteConnection(Connection::Ptr connection);

protected:
    std::vector<BoxPtr> boxes_;
    std::vector<Connector*> connectors_;
    std::vector<Connection::Ptr> connections;

    QTimer* timer_;
};

}

#endif // GRAPH_H
