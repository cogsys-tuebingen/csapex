#ifndef BOX_H
#define BOX_H

/// COMPONENT
#include <csapex/model/boxed_object.h>
#include <csapex/model/memento.h>
#include <csapex/command/command.h>
#include <csapex/view/selectable.h>
#include <csapex/model/graph.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <QIcon>
#include <QMenu>
#include <QMutex>
#include <QWidget>
#include <yaml-cpp/yaml.h>

/// FORWARD DECLARATIONS
namespace Ui
{
class Box;
}

namespace csapex
{


class Box : public QWidget, public Selectable
{
    Q_OBJECT

    friend class DesignerIO;
    friend class GraphIO;
    friend class Graph;
    friend class NodeWorker;
    friend class command::MoveBox;
    friend class command::AddConnector;

public:
    typedef boost::shared_ptr<Box> Ptr;
    static const Ptr NullPtr;

public:
    struct State : public Memento {
        typedef boost::shared_ptr<State> Ptr;

        State()
            : parent(NULL), minimized(false), enabled(true)
        {}
        State(Box* parent)
            : parent(parent), minimized(false), enabled(true)
        {}

        void copyFrom (const Ptr &rhs);

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);

        Box* parent;

        mutable Memento::Ptr boxed_state;

        std::string uuid_;
        std::string label_;

        QPoint pos;

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

    void moveEvent(QMoveEvent*);
    void triggerPlaced();

    void selectEvent();
    void deselectEvent();

    void keyPressEvent(QKeyEvent * e);

    virtual void init(const QPoint& pos);

    template <typename T>
    ConnectorIn* addInput(const std::string& label, bool optional = false) {
        return addInput(T::make(), label, optional);
    }

    template <typename T>
    ConnectorOut* addOutput(const std::string& label) {
        return addOutput(T::make(), label);
    }

    ConnectorIn* addInput(ConnectionTypePtr type, const std::string& label, bool optional);
    ConnectorOut* addOutput(ConnectionTypePtr type, const std::string& label);

    void addInput(ConnectorIn* in) __attribute__ ((deprecated));
    void addOutput(ConnectorOut* out) __attribute__ ((deprecated));

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

    std::string getType() const;

    void setLabel(const std::string& label);
    void setLabel(const QString& label);
    std::string getLabel() const;

    void setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

    virtual bool hasSubGraph();
    virtual Graph::Ptr getSubGraph();

    Command::Ptr removeAllConnectionsCmd();

    YAML::Emitter& save(YAML::Emitter& out) const;
    void read(YAML::Node& doc);

    bool isMinimizedSize() const;

    void setSynchronizedInputs(bool sync);

    CommandDispatcher* getCommandDispatcher() const;
    void setCommandDispatcher(CommandDispatcher* d);

    virtual void fillContextMenu(QMenu* menu, std::map<QAction *, boost::function<void()> > &handler);

    NodeWorker* getNodeWorker();


    bool isError() const;
    Displayable::ErrorLevel errorLevel() const;
    std::string errorMessage() const;
    void setError(bool e, const std::string& msg, Displayable::ErrorLevel level = Displayable::EL_ERROR);

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
    void showProfiling();
    void tick();

    void messageProcessed();
    void showContextMenu(const QPoint& pos);

Q_SIGNALS:
    void placed();
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

    void showContextMenuForBox(Box* box, const QPoint& pos);


protected:
    void connectConnector(Connector* c);
    void disconnectConnector(Connector* c);

    void resizeEvent(QResizeEvent * e);

    void registerInput(ConnectorIn* in);
    void registerOutput(ConnectorOut* out);


protected:
    Ui::Box* ui;

    CommandDispatcher* dispatcher_;

    BoxedObject::Ptr content_;
    State::Ptr state;

    std::vector<ConnectorIn*> input;

    std::vector<ConnectorOut*> output;


    QMutex worker_mutex_;

    QThread* private_thread_;
    NodeWorker* worker_;

    bool down_;
    QPoint start_drag_;
    QPoint start_drag_global_;

    QIcon minimize_icon_;
    QIcon maximize_icon_;

    int next_sub_id_;

    QPoint key_point;

    bool profiling_;
    ProfilingWidget* prof;
};

}
#endif // BOX_H
