#ifndef BOX_H
#define BOX_H

/// COMPONENT
#include "connector_in.h"
#include "connector_out.h"
#include "memento.h"

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


namespace vision_evaluator
{

class BoxedObject;


struct BoxWorker : public QObject
{
    Q_OBJECT

public:
    BoxWorker(Box* parent)
        : parent_(parent)
    {}

public Q_SLOTS:
    void forwardMessage(ConnectorIn* source);
    void tick();
    Box* parent();

private:
    Box* parent_;
};

class Box : public QWidget
{
    Q_OBJECT

    friend class DesignerIO;
    friend class BoxWorker;

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
        std::string type_;

        bool minimized;
        bool enabled;
    };

public:
    static const QString MIME;
    static const QString MIME_MOVE;


public:
    Box(BoxedObject* content, const std::string& uuid = "", QWidget* parent = 0);
    virtual ~Box();

    void stop();

    virtual void mousePressEvent(QMouseEvent* e);
    virtual QPixmap makePixmap(const std::string& label);

    void moveEvent(QMoveEvent*);
    void registered();

    void focusInEvent(QFocusEvent * e);
    void focusOutEvent(QFocusEvent * e);

    void keyPressEvent(QKeyEvent * e);

    virtual void init(const QPoint& pos);
    BoxedObject* getContent();

    void addInput(ConnectorIn* in);
    void addOutput(ConnectorOut* out);

    void removeInput(ConnectorIn *in);
    void removeOutput(ConnectorOut *out);

    int  countInputs();
    int  countOutputs();

    ConnectorIn* getInput(const unsigned int index);
    ConnectorOut *getOutput(const unsigned int index);

    ConnectorIn* getInput(const std::string& uuid);
    ConnectorOut* getOutput(const std::string& uuid);

    void setUUID(const std::string& uuid);
    std::string UUID() const;

    void setType(const std::string& type);
    std::string getType() const;

    Memento::Ptr getState() const;
    void setState(Memento::Ptr memento);

    Command::Ptr removeAllConnectionsCmd();
    Command::Ptr removeAllOutputsCmd();
    Command::Ptr removeAllInputsCmd();

    YAML::Emitter& save(YAML::Emitter& out) const;

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
    void refreshStylesheet();
    void eventModelChanged();
    void killContent();

Q_SIGNALS:
    void toggled(bool);
    void moved(Box*);
    void connectorCreated(Connector*);
    void connectionFormed(ConnectorOut*, ConnectorIn*);
    void connectionDestroyed(ConnectorOut*, ConnectorIn*);

    void messageSent(ConnectorOut* source);
    void messageArrived(ConnectorIn* source);
    void connectionInProgress(Connector*, Connector*);
    void connectionDone();
    void connectionStart();
    void connectorEnabled(Connector* source);
    void connectorDisabled(Connector* source);

private:
    void connectConnector(Connector* c);
    void disconnectConnector(Connector* c);

private:
    Ui::Box* ui;

    State::Ptr state;
    BoxedObject* content_;

    std::vector<ConnectorIn*> input;
    std::vector<ConnectorOut*> output;  

    QMutex worker_mutex_;

    QThread* private_thread_;
    QTimer* timer_;
    BoxWorker* worker_;

    bool down_;

    QIcon minimize_icon_;
    QIcon maximize_icon_;
};

inline YAML::Emitter& operator << (YAML::Emitter& out, const Box& box)
{
    return box.save(out);
}

}
#endif // BOX_H
