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
#include <QBoxLayout>
#include <QLabel>

/// FORWARD DECLARATIONS
namespace Ui
{
class Box;
}

namespace csapex
{


class NodeBox : public QWidget, public Selectable
{
    Q_OBJECT

    friend class DesignerIO;
    friend class GraphIO;
    friend class Graph;
    friend class NodeWorker;
    friend class Node;
    friend class command::MoveBox;
    friend class command::AddConnector;
    friend class BoxSelectionmanager;

public:
    typedef boost::shared_ptr<NodeBox> Ptr;

public:
    static const QString MIME;
    static const QString MIME_MOVE;

public:
    /// CONSTRUCTION
    NodeBox(NodePtr content, NodeAdapterPtr adapter, QWidget* parent = 0);
    virtual ~NodeBox();
    void construct();
    void init();

    /// ACCESSORS
    Node* getNode();
    NodeAdapterPtr getNodeAdapter();

    CommandDispatcher* getCommandDispatcher() const;
    void setCommandDispatcher(CommandDispatcher* d);

    virtual bool hasSubGraph();
    virtual Graph::Ptr getSubGraph();

    void setLabel(const std::string& label);
    void setLabel(const QString& label);
    std::string getLabel() const;

    bool isMinimizedSize() const;

    bool isError() const;
    ErrorState::ErrorLevel errorLevel() const;
    std::string errorMessage() const;

    /// UI
    virtual void fillContextMenu(QMenu* menu, std::map<QAction *, boost::function<void()> > &handler);

    QBoxLayout* getInputLayout();
    QBoxLayout* getOutputLayout();

    /// UI CALLBACKS
//    virtual void mousePressEvent(QMouseEvent* e);
//    virtual void mouseReleaseEvent(QMouseEvent* e);
//    virtual void mouseMoveEvent(QMouseEvent* e);

    void moveEvent(QMoveEvent*);
    void triggerPlaced();

    void selectEvent();
    void deselectEvent();

    void keyPressEvent(QKeyEvent * e);

    void stop();

protected:
    void setupUi();
    void startDrag(QPoint offset);
    void paintEvent(QPaintEvent* e);
    bool eventFilter(QObject*, QEvent*);
    void updateFlippedSides();

public Q_SLOTS:
    void setupUiAgain();

    void deleteBox();
    void minimizeBox(bool minimize);
    void enableContent(bool enable);
    void refreshStylesheet();
    void eventModelChanged();
    void killContent();
    void flipSides();
    void showProfiling();

    void updateInformation(Graph* graph);
    void contextMenuEvent(QContextMenuEvent* e);

    void registerEvent(Connectable*);
    void unregisterEvent(Connectable*);

    void nodeStateChanged();
    void enabledChange(bool val);

    void setError(bool e, const std::string& msg);
    void setError(bool e, const std::string& msg, int level);

Q_SIGNALS:
    void placed();
    void toggled(bool);
    void flipped(bool);
    void minimized(bool);

    void moveRequest(NodeBox*, QPoint);
    void moved(NodeBox*, int dx, int dy);

    void changed(NodeBox*);
    void clicked(NodeBox*);
    void moveSelectionToBox(NodeBox*);

    void showContextMenuForBox(NodeBox* box, const QPoint& pos);


protected:
    void resizeEvent(QResizeEvent * e);

    void registerInputEvent(ConnectorIn* in);
    void registerOutputEvent(ConnectorOut* out);


protected:
    Ui::Box* ui;

    NodePtr node_;
    NodeAdapterPtr adapter_;

    bool down_;
    QPoint start_drag_;
    QPoint start_drag_global_;

    QIcon minimize_icon_;
    QIcon maximize_icon_;

    QPoint key_point;

    QLabel* info_compo;

    bool profiling_;
    ProfilingWidget* prof;

    bool is_placed_;
};

}
#endif // BOX_H
