#ifndef BOX_H
#define BOX_H

/// COMPONENT
#include <csapex/model/memento.h>
#include <csapex/command/command.h>
#include <csapex/view/selectable.h>
#include <csapex/model/graph.h>
#include <csapex/csapex_fwd.h>
#include <csapex/model/error_state.h>

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
    friend class Node;
    friend class command::MoveBox;
    friend class command::AddConnector;

public:
    typedef boost::shared_ptr<Box> Ptr;

public:
    static const QString MIME;
    static const QString MIME_MOVE;

public:
    Box(BoxedObject *content, QWidget* parent = 0);
    Box(Node* content, NodeAdapterPtr adapter, QWidget* parent = 0);
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

    Node* getNode();

    std::string UUID() const;

    std::string getType() const;

    void setLabel(const std::string& label);
    void setLabel(const QString& label);
    std::string getLabel() const;

    virtual bool hasSubGraph();
    virtual Graph::Ptr getSubGraph();

    bool isMinimizedSize() const;

    CommandDispatcher* getCommandDispatcher() const;
    void setCommandDispatcher(CommandDispatcher* d);

    virtual void fillContextMenu(QMenu* menu, std::map<QAction *, boost::function<void()> > &handler);

    bool isError() const;
    ErrorState::ErrorLevel errorLevel() const;
    std::string errorMessage() const;
    void setError(bool e, const std::string& msg, ErrorState::ErrorLevel level = ErrorState::EL_ERROR);

    void construct(Node* node);
    void init();

protected:
    void startDrag(QPoint offset);
    void paintEvent(QPaintEvent* e);
    bool eventFilter(QObject*, QEvent*);
    void enabledChange(bool val);

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

    void showContextMenu(const QPoint& pos);

    void registerEvent(Connector*);
    void unregisterEvent(Connector*);

    void nodeStateChanged();


Q_SIGNALS:
    void placed();
    void toggled(bool);
    void moved(Box*, int dx, int dy);
    void changed(Box*);
    void clicked(Box*);
    void moveSelectionToBox(Box*);

    void showContextMenuForBox(Box* box, const QPoint& pos);


protected:
    void resizeEvent(QResizeEvent * e);

    void registerInputEvent(ConnectorIn* in);
    void registerOutputEvent(ConnectorOut* out);


protected:
    Ui::Box* ui;

    Node* node_;
    NodeAdapterPtr adapter_;
    NodeAdapter* adapter_shared_;

    bool down_;
    QPoint start_drag_;
    QPoint start_drag_global_;

    QIcon minimize_icon_;
    QIcon maximize_icon_;

    QPoint key_point;

    bool profiling_;
    ProfilingWidget* prof;
};

}
#endif // BOX_H
