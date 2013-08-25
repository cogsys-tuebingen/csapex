#ifndef BOX_H
#define BOX_H

/// COMPONENT
#include <csapex/boxed_object.h>
#include <csapex/memento.h>
#include <csapex/command.h>
#include <csapex/selectable.h>
#include <csapex/graph.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QWidget>
#include <QIcon>
#include <QMutex>
#include <yaml-cpp/yaml.h>

/// FORWARD DECLARATIONS
namespace Ui
{
class Box;
}

namespace csapex
{

struct BoxWorker : public QObject
{
    Q_OBJECT

public:
    BoxWorker(Box* parent)
        : parent_(parent)
    {}

public Q_SLOTS:
    void forwardMessage(Connector* source);
    void tick();
    Box* parent();

private:
    Box* parent_;
};

class Box : public QWidget, public Selectable
{
    Q_OBJECT

    friend class DesignerIO;
    friend class GraphIO;
    friend class Graph;
    friend class BoxWorker;

public:
    typedef boost::shared_ptr<Box> Ptr;
    static const Ptr NullPtr;

public:
    struct MoveOffset : public QObjectUserData {
        MoveOffset(const QPoint& o)
            : value(o)
        {}

        QPoint value;
    };

    struct State : public Memento {
        typedef boost::shared_ptr<State> Ptr;

        State()
            : parent(NULL), minimized(false), enabled(true)
        {}
        State(Box* parent)
            : parent(parent), minimized(false), enabled(true)
        {}

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);

        Box* parent;

        mutable Memento::Ptr boxed_state;

        std::string uuid_;
        std::string label_;
        std::string type_;

        bool minimized;
        bool enabled;
    };

public:
    static const QString MIME;
    static const QString MIME_MOVE;


public:
    Box(BoxedObject::Ptr content, const std::string& uuid = "", QWidget* parent = 0);
    virtual ~Box();

    void stop();

    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseReleaseEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent* e);

    virtual QPixmap makePixmap(const std::string& label);

    void moveEvent(QMoveEvent*);
    void registered();

    void selectEvent();
    void deselectEvent();

    void keyPressEvent(QKeyEvent * e);

    virtual void init(const QPoint& pos);
    BoxedObject::Ptr getContent();

    void addInput(ConnectorIn* in);
    void addOutput(ConnectorOut* out);

    int nextInputId();
    int nextOutputId();

    void removeInput(ConnectorIn *in);
    void removeOutput(ConnectorOut *out);

    int  countInputs();
    int  countOutputs();

    ConnectorIn* getInput(const unsigned int index);
    ConnectorOut *getOutput(const unsigned int index);

    ConnectorIn* getInput(const std::string& uuid);
    ConnectorOut* getOutput(const std::string& uuid);

    std::string UUID() const;

    void setType(const std::string& type);
    std::string getType() const;

    void setLabel(const std::string& label);
    void setLabel(const QString& label);
    std::string getLabel() const;

    void setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

    virtual bool hasSubGraph();
    virtual Graph::Ptr getSubGraph();

    Command::Ptr removeAllConnectionsCmd();
    Command::Ptr removeAllOutputsCmd();
    Command::Ptr removeAllInputsCmd();

    YAML::Emitter& save(YAML::Emitter& out) const;

protected:

protected:
    void startDrag(QPoint offset);
    void paintEvent(QPaintEvent* e);
    bool eventFilter(QObject*, QEvent*);
    void enabledChange(bool val);
    void makeThread();

public Q_SLOTS:
    void deleteBox();
    void minimizeBox(bool minimize);
    void enableContent(bool enable);
    void enableIO(bool enable);
    void setIOError(bool error);
    void refreshStylesheet();
    void eventModelChanged();
    void killContent();
    void tick();

Q_SIGNALS:
    void toggled(bool);
    void moved(Box*, int dx, int dy);
    void changed(Box*);
    void clicked(Box*);
    void tickRequest();
    void moveSelectionToBox(Box*);

    void connectorCreated(Connector*);
    void connectionFormed(Connector*, Connector*);
    void connectionDestroyed(Connector*, Connector*);

    void connectionInProgress(Connector*, Connector*);
    void connectionDone();
    void connectionStart();
    void connectorEnabled(Connector* source);
    void connectorDisabled(Connector* source);

protected:
    void connectConnector(Connector* c);
    void disconnectConnector(Connector* c);

    void resizeEvent(QResizeEvent * e);

protected:
    Ui::Box* ui;

    Graph *graph_;
    State::Ptr state;
    BoxedObject::Ptr content_;

    std::vector<ConnectorIn*> input;
    std::vector<ConnectorOut*> output;  

    QMutex worker_mutex_;

    QThread* private_thread_;
    BoxWorker* worker_;

    bool down_;
    QPoint start_drag_;
    QPoint start_drag_global_;

    QIcon minimize_icon_;
    QIcon maximize_icon_;

    int next_sub_id_;
};

}
#endif // BOX_H
