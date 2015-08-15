#ifndef BOX_H
#define BOX_H

/// COMPONENT
#include <csapex/model/memento.h>
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/csapex_fwd.h>
#include <csapex/model/error_state.h>
#include <csapex/model/node_worker.h>

/// SYSTEM
#include <memory>
#include <functional>
#include <QIcon>
#include <QMenu>
#include <QWidget>
#include <QBoxLayout>
#include <QLabel>

/// FORWARD DECLARATIONS
namespace Ui
{
class Box;
}

class QSizeGrip;

namespace csapex
{


class NodeBox : public QWidget
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
    typedef std::shared_ptr<NodeBox> Ptr;

public:
    static const QString MIME;

public:
    /// CONSTRUCTION
    NodeBox(Settings& settings, NodeWorkerPtr content, NodeAdapterPtr adapter, QIcon icon, QWidget* parent = 0);
    virtual ~NodeBox();
    void construct();
    void init();

    /// ACCESSORS
    Node* getNode();
    NodeWorker* getNodeWorker();
    NodeAdapterPtr getNodeAdapter();

    virtual bool hasSubGraph();
    virtual Graph::Ptr getSubGraph();

    void setLabel(const std::string& label);
    void setLabel(const QString& label);
    std::string getLabel() const;

    bool isMinimizedSize() const;
    bool isFlipped() const;

    bool isError() const;
    ErrorState::ErrorLevel errorLevel() const;
    std::string errorMessage() const;

    QBoxLayout* getInputLayout();
    QBoxLayout* getOutputLayout();
    QBoxLayout* getSlotLayout();
    QBoxLayout* getTriggerLayout();

    /// UI CALLBACKS
    void moveEvent(QMoveEvent*);
    void triggerPlaced();

    void setSelected(bool selected);

    void keyPressEvent(QKeyEvent * e);

    void stop();

protected:
    void setupUi();
    void paintEvent(QPaintEvent* e);
    bool eventFilter(QObject*, QEvent*);


public:
    void updateBoxInformation(Graph* graph);

public Q_SLOTS:
    void setupUiAgain();


    void getInformation();
    void minimizeBox();
    void refreshStylesheet();
    void killContent();
    void flipSides();
    void showProfiling(bool show);

    void updateComponentInformation(Graph* graph);
    void updateThreadInformation();
    void contextMenuEvent(QContextMenuEvent* e);

    void registerEvent(Connectable*);
    void unregisterEvent(Connectable*);

    void nodeStateChangedEvent();
    void enabledChangeEvent(bool val);

    void updateVisuals();

Q_SIGNALS:
    void placed();
    void toggled(bool);
    void flipped(bool);
    void minimized(bool);

    void renameRequest(NodeBox*);
    void helpRequest(NodeBox*);

    void moveRequest(NodeBox*, QPoint);
    void moved(NodeBox*, int dx, int dy);

    void changed(NodeBox*);
    void clicked(NodeBox*);
    void moveSelectionToBox(NodeBox*);

    void showContextMenuForBox(NodeBox* box, const QPoint& pos);

    void updateVisualsRequest();

    void nodeStateChanged();
    void enabledChange(bool val);

protected:
    void resizeEvent(QResizeEvent * e);

    void registerInputEvent(Input* in);
    void registerOutputEvent(Output* out);

protected:
    Ui::Box* ui;
    QSizeGrip* grip_;

    Settings& settings_;

    NodeWorkerWeakPtr node_worker_;
    NodeAdapterPtr adapter_;

    QIcon icon_;

    bool down_;
    QPoint start_drag_;
    QPoint start_drag_global_;

    QIcon minimize_icon_;
    QIcon maximize_icon_;

    QLabel* info_exec;
    QLabel* info_compo;
    QLabel* info_thread;
    QLabel* info_error;

    bool is_placed_;
};

}
#endif // BOX_H
