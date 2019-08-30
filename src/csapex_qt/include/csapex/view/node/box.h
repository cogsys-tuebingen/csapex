#ifndef BOX_H
#define BOX_H

/// COMPONENT
#include <csapex/view/view_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/serialization/serializable.h>
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/error_state.h>
#include <csapex/model/node_facade.h>
#include <csapex/model/connector_type.h>
#include <csapex/model/connector_description.h>
#include <csapex_qt/export.h>
#include <csapex/view/utility/qobserver.h>

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
class CSAPEX_QT_EXPORT NodeBox : public QWidget, public Observer
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    typedef std::shared_ptr<NodeBox> Ptr;

    QString cssClass()
    {
        return QString("NodeBox");
    }

public:
    /// CONSTRUCTION
    NodeBox(Settings& settings, NodeFacadePtr node_facade, QIcon icon, GraphView* parent = 0);

    void setAdapter(NodeAdapterPtr adapter);

    virtual ~NodeBox();
    virtual void construct();
    void destruct();
    virtual void init();

    /// MODIFIER
    Port* createPort(ConnectorPtr connector, QBoxLayout* layout);
    void removePort(ConnectorWeakPtr connector);

    bool isGraph() const;

    /// ACCESSORS
    NodeFacadePtr getNodeFacade() const;
    NodeAdapterPtr getNodeAdapter() const;

    GraphView* getGraphView() const;

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
    QBoxLayout* getEventLayout();

    /// UI CALLBACKS
    void moveEvent(QMoveEvent*);

    bool isSelected() const;
    virtual void setSelected(bool selected);

    void keyPressEvent(QKeyEvent* e);

    void stop();

protected:
    void setupUi();
    void paintEvent(QPaintEvent* e);
    bool eventFilter(QObject*, QEvent*);

public:
    void updateBoxInformation(GraphFacade* graph);

public Q_SLOTS:
    void getInformation();
    void triggerMinimized();
    void triggerEnabledChanged();
    void changeColor();
    void refreshStylesheet();
    virtual void refreshTopLevelStylesheet();
    void triggerFlipSides();
    void showProfiling(bool show);

    virtual void updateComponentInformation(GraphFacade* graph);
    virtual void updateThreadInformation();
    virtual void updateFrequencyInformation();
    void contextMenuEvent(QContextMenuEvent* e);

    void nodeStateChangedEvent();
    void enabledChangeEvent(bool val);

    void updateVisuals();
    void updatePosition();

Q_SIGNALS:
    void toggled(bool);
    void flipped(bool);
    void minimized(bool);

    void renameRequest(NodeBox*);
    void helpRequest(NodeBox*);

    void clicked(NodeBox*);
    void moveSelectionToBox(NodeBox*);

    void showContextMenuForBox(NodeBox* box, const QPoint& pos);

    void showSubGraphRequest(UUID graphid);

    void updateVisualsRequest();

    void nodeStateChanged();
    void enabledChange(bool val);

    void portAdded(Port*);
    void portRemoved(Port*);

    void createPortRequest(ConnectorDescription request);
    void createPortAndConnectRequest(ConnectorDescription request, ConnectorPtr);
    void createPortAndMoveRequest(ConnectorDescription request, ConnectorPtr);

protected:
    void setStyleForId(QLabel* label, int id);
    void resizeEvent(QResizeEvent* e);

    QString getNodeState();

    virtual void updateStylesheetColor();

    virtual void startResize();
    virtual void stopResize();

protected:
    GraphView* parent_;

    QObserver observer_;

    Ui::Box* ui;
    QSizeGrip* grip_;

    Settings& settings_;

    NodeFacadePtr node_facade_;
    NodeAdapterPtr adapter_;

    std::unordered_map<UUID, Port*, UUID::Hasher> port_map_;

    QIcon icon_;

    QIcon minimize_icon_;
    QIcon maximize_icon_;

    QLabel* info_exec;
    QLabel* info_compo;
    QLabel* info_thread;
    QLabel* info_frequency;
    QLabel* info_error;

    bool initialized_;
    bool moved_;

    QTimer* frequency_timer_;

    ExecutionState cached_state_;
    using stamp_t = std::chrono::high_resolution_clock::time_point;
    static const std::chrono::milliseconds cache_update_rate_;
    stamp_t last_state_request_;
};

}  // namespace csapex
#endif  // BOX_H
