#ifndef GRAPH_H
#define GRAPH_H

/// COMPONENT
#include <csapex/model/connection.h>
#include <csapex/command/meta.h>
#include <csapex/csapex_fwd.h>

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
    friend class command::AddBox;
    friend class command::AddConnection;
    friend class command::DeleteConnection;
    friend class command::DeleteBox;
    friend class Overlay;
    friend class BoxMeta;

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

    Graph::Ptr findSubGraph(const std::string& uuid);

    BoxPtr findBox(const std::string& uuid) __attribute__ ((deprecated));
    BoxPtr findBoxNoThrow(const std::string& uuid) __attribute__ ((deprecated));
    BoxPtr findConnectorOwner(const std::string &uuid) __attribute__ ((deprecated));

    NodePtr findNode(const std::string& uuid);
    NodePtr findNodeNoThrow(const std::string& uuid);
    NodePtr findNodeForConnector(const std::string &uuid);


    Connector* findConnector(const std::string &uuid);


    bool handleConnectionSelection(int id, bool add);
    Command::Ptr deleteConnectionByIdCommand(int id);

    Command::Ptr deleteConnectionFulcrumCommand(int connection, int fulcrum);

    Command::Ptr deleteAllConnectionFulcrumsCommand(int connection);
    Command::Ptr deleteAllConnectionFulcrumsCommand(Connection::Ptr connection);

    Command::Ptr deleteConnectionById(int id);
    Command::Ptr deleteSelectedConnections();
    int noSelectedConnections();

    void handleBoxSelection(Box* box, bool add);
    Command::Ptr deleteSelectedBoxes();
    Command::Ptr groupSelectedBoxes();
    void selectBox(Box* box, bool add = false);
    void deselectBoxes();
    int noSelectedBoxes();

    Connection::Ptr getConnectionWithId(int id);
    Connection::Ptr getConnection(Connection::Ptr);
    int getConnectionId(Connection::Ptr);

    std::string makeUUID(const std::string& name);

    Command::Ptr clear();

    TemplatePtr toTemplate(const std::string& name) const;

    bool hasSelectedBox() const;
    std::vector<BoxPtr> getSelectedBoxes() const;
    void fillContextMenuForSelection(QMenu* menu, std::map<QAction *, boost::function<void()> > &handler);
    void foreachBox(boost::function<void (Box*)> f, boost::function<bool (Box*)> pred);

public Q_SLOTS:
    void reset();
    void tick();
    void clearSelection();
    void selectAll();

    void toggleBoxSelection(Box* box);
    void boxMoved(Box* box, int dx, int dy);

    Command::Ptr moveSelectedBoxes(const QPoint& delta);

Q_SIGNALS:
    void stateChanged();
    void dirtyChanged(bool);

    void boxAdded(Box*);
    void boxDeleted(Box*);
    void connectionAdded(Connection*);
    void connectionDeleted(Connection*);

private:
    void deselectConnections();
    void deselectConnectionById(int id);
    void selectConnectionById(int id, bool add = false);
    bool isConnectionWithIdSelected(int id);

    TemplatePtr convertSelectionToTemplate(std::vector<std::pair<std::string, std::string> > &connections) const;
    TemplatePtr generateTemplate(TemplatePtr templ, std::vector<std::pair<std::string, std::string> > &connections, bool only_selected) const;


private: /// ONLY COMMANDS / NOT UNDOABLE
    void addBox(BoxPtr box);
    void deleteBox(const std::string &uuid);

    bool addConnection(Connection::Ptr connection);
    void deleteConnection(Connection::Ptr connection);

protected:
    std::vector<BoxPtr> boxes_;
    std::vector<Connector*> connectors_;
    std::vector<Connection::Ptr> visible_connections;

    CommandDispatcher* dispatcher_;

    std::map<std::string, int> uuids;

    QTimer* timer_;
};

}

#endif // GRAPH_H
