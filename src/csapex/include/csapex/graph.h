#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/connection.h>

/// SYSTEM
#include <QObject>
#include <QTimer>

namespace csapex {

namespace command {
class AddBox;
class AddConnection;
class AddConnectionForwarding;
class DeleteConnection;
class DeleteBox;
}

class Box;

class Graph : public QObject
{
    Q_OBJECT

    friend class GraphIO;
    friend class command::AddBox;
    friend class command::AddConnection;
    friend class command::AddConnectionForwarding;
    friend class command::DeleteConnection;
    friend class command::DeleteBox;
    friend class Overlay;
    friend class BoxMeta;

public:
    static const std::string namespace_separator;

public:
    Graph();
    virtual ~Graph();

    bool isDirty();

    bool canUndo();
    bool canRedo();

    Box* findBox(const std::string& uuid, const std::string &ns = "");
    Box* findConnectorOwner(const std::string &uuid, const std::string &ns = "");
    Connector* findConnector(const std::string &uuid, const std::string &ns = "");

    bool handleConnectionSelection(int id, bool add);
    void deleteConnectionById(int id);
    void deleteSelectedConnections();
    int noSelectedConnections();

    void handleBoxSelection(Box* box, bool add);
    void deleteSelectedBoxes();
    void selectBox(Box* box, bool add = false);
    void deselectBoxes();
    int noSelectedBoxes();

public Q_SLOTS:
    void undo();
    void redo();
    void clear();
    void tick();


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

private: /// ONLY COMMANDS / NOT UNDOABLE
    void addBox(Box* box);
    void deleteBox(Box* box);

    bool addConnection(Connection::Ptr connection);
    void deleteConnection(Connection::Ptr connection);

protected:
    std::vector<Box*> boxes_;
    std::vector<Connector*> connectors_;
    std::vector<Connection::Ptr> connections;

    QTimer* timer_;
};

}

#endif // GRAPH_H
