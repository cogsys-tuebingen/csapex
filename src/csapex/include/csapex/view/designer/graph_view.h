#ifndef GRAPH_VIEW_H
#define GRAPH_VIEW_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/view/view_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/core/core_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/view/designer/designer_styleable.h>
#include <csapex/utility/slim_signal.hpp>
#include <csapex/utility/uuid.h>
#include <csapex/model/connector_type.h>
#include <csapex/utility/create_connector_request.h>
#include <csapex/view/csapex_view_core.h>
#include <csapex/profiling/profilable.h>
#include <csapex/model/execution_mode.h>
#include <csapex/serialization/snippet.h>

/// SYSTEM
#include <QGraphicsView>
#include <map>
#include <QTimer>
#include <unordered_map>

namespace csapex
{

class NodeFactory;
class GraphViewContextMenu;

class CSAPEX_QT_EXPORT GraphView : public QGraphicsView, public Profilable
{
    friend class GraphViewContextMenu;

    Q_OBJECT

    Q_PROPERTY(QColor lineColor         READ lineColor         WRITE setLineColor)
    Q_PROPERTY(QColor lineColorError    READ lineColorError    WRITE setLineColorError)
    Q_PROPERTY(QColor lineColorBlocked  READ lineColorBlocked  WRITE setLineColorBlocked)
    Q_PROPERTY(QColor lineColorDisabled READ lineColorDisabled WRITE setLineColorDisabled)
    Q_PROPERTY(int lineWidth            READ lineWidth         WRITE setLineWidth)

    Q_PROPERTY(QColor balloonColor         READ balloonColor         WRITE setBalloonColor)

    void setLineColor(const QColor& c) { view_core_.getStyle().setLineColor(c); }
    void setLineColorError(const QColor& c) { view_core_.getStyle().setLineColorError(c); }
    void setLineColorBlocked(const QColor& c) { view_core_.getStyle().setLineColorBlocked(c); }
    void setLineColorDisabled(const QColor& c) { view_core_.getStyle().setLineColorDisabled(c);}
    void setLineWidth(int width) { view_core_.getStyle().setLineWidth(width); }
    void setBalloonColor(const QColor& c) { view_core_.getStyle().setBalloonColor(c); }

    QColor lineColor() const { return view_core_.getStyle().lineColor(); }
    QColor lineColorError() const { return view_core_.getStyle().lineColorError(); }
    QColor lineColorBlocked() const { return view_core_.getStyle().lineColorBlocked(); }
    QColor lineColorDisabled() const { return view_core_.getStyle().lineColorDisabled();}
    int lineWidth() const { return view_core_.getStyle().lineWidth(); }
    QColor balloonColor() const { return view_core_.getStyle().balloonColor(); }

public:
    GraphView(csapex::GraphFacadePtr graph_facade, CsApexViewCore& view_core, QWidget *parent = nullptr);
    ~GraphView();

    DesignerScene* designerScene();
    std::vector<NodeBox*> boxes();
    std::vector<NodeBox*> getSelectedBoxes() const;
    std::vector<UUID> getSelectedUUIDs() const;
    CommandPtr deleteSelected();

    NodeBox* getBox(const csapex::UUID& node_id);
    MovableGraphicsProxyWidget* getProxy(const csapex::UUID& node_id);

    GraphFacade* getGraphFacade() const;

    CsApexViewCore& getViewCore() const;

    void focusOnNode(const csapex::UUID& uuid);

    void resizeEvent(QResizeEvent *event);
    void scrollContentsBy(int dx, int dy);

    void keyPressEvent(QKeyEvent* e);
    void keyReleaseEvent(QKeyEvent* e);

    void mousePressEvent(QMouseEvent* e);
    void mouseReleaseEvent(QMouseEvent* e);

    void wheelEvent(QWheelEvent* we);

    void mouseMoveEvent(QMouseEvent* me);
    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);
    void dragLeaveEvent(QDragLeaveEvent * e);

    void drawForeground(QPainter* painter, const QRectF& rect);
    void paintEvent(QPaintEvent* e);

    void nodeAdded(NodeWorkerPtr node_worker);
    void nodeRemoved(NodeHandlePtr node_handle);

    void startPlacingBox(const std::string& type, NodeStatePtr state, const QPoint &offset = QPoint(0,0));
    void startCloningSelection(NodeBox *handle, const QPoint &offset = QPoint(0,0));

    virtual void useProfiler(std::shared_ptr<Profiler> profiler) override;

Q_SIGNALS:
    void selectionChanged();
    void viewChanged();

    void boxAdded(NodeBox* box);
    void boxRemoved(NodeBox* box);

    void startProfilingRequest(NodeWorker* box);
    void stopProfilingRequest(NodeWorker *box);

    void triggerConnectorCreated(ConnectablePtr connector);
    void triggerConnectorRemoved(ConnectablePtr connector);

public Q_SLOTS:
    void showNodeInsertDialog();

    void addBox(NodeBox* box);
    void removeBox(NodeBox* box);

    void createPort(CreateConnectorRequest request);
    void createPortAndConnect(CreateConnectorRequest request, Connectable* from);
    void createPortAndMove(CreateConnectorRequest request, Connectable* from);

    void addPort(Port* port);
    void removePort(Port* port);

    void renameBox(NodeBox* box);

    void connectorCreated(ConnectablePtr connector);
    void connectorRemoved(ConnectablePtr connector);
    void connectorSignalAdded(ConnectablePtr connector);
    void connectorMessageAdded(ConnectablePtr connector);

    void centerOnPoint(QPointF point);

    void movedBoxes(double dx, double dy);

    void overwriteStyleSheet(const QString& stylesheet);
    void updateBoxInformation();

    void contextMenuEvent(QContextMenuEvent* e);
    void showContextMenuGlobal(const QPoint& pos);
    void showContextMenuForSelectedNodes(NodeBox *box, const QPoint& pos);

    void startProfiling(NodeWorker* box);
    void stopProfiling(NodeWorker *box);

    void reset();

    void resetZoom();
    void zoomIn();
    void zoomOut();
    void zoom(double factor);
    void zoomAt(QPointF point, double factor);
    void animateZoom();

    void animateScroll();

    void enableSelection(bool enabled);

    void updateSelection();
    void selectAll();

    void copySelected();
    void paste();

    void groupSelected();
    void ungroupSelected();

    void makeSnippetFromSelected();

    void showPreview(Port* port);
    void stopPreview();

private:
    void createNodes(const QPoint& global_pos, const std::string &type, const std::string &mime);

    void setupWidgets();

    void flipBox();
    void setExecutionMode(ExecutionMode mode);
    void setLoggerLevel(int level);
    void chooseColor();
    void minimizeBox(bool mini);
    void muteBox(bool muted);
    void deleteBox();
    void morphNode();
    void usePrivateThreadFor();
    void useDefaultThreadFor();
    void switchToThread(int group_id);
    void createNewThreadGroupFor();

    void showProfiling(bool visible);

    Snippet serializeSelection() const;

private:
    CsApexCore& core_;
    CsApexViewCore& view_core_;

    DesignerScene* scene_;

    GraphFacadePtr graph_facade_;

    PortPanel* inputs_widget_;
    QGraphicsProxyWidget* inputs_widget_proxy_;
    PortPanel* outputs_widget_;
    QGraphicsProxyWidget* outputs_widget_proxy_;
    PortPanel* slots_widget_;
    QGraphicsProxyWidget* slots_widget_proxy_;
    PortPanel* events_widget_;
    QGraphicsProxyWidget* triggers_widget_proxy_;

    std::map<NodeWorker*, std::vector<csapex::slim_signal::ScopedConnection>> worker_connections_;
    std::map<NodeHandle*, std::vector<csapex::slim_signal::ScopedConnection>> handle_connections_;
    std::vector<csapex::slim_signal::ScopedConnection> scoped_connections_;

    std::vector<NodeBox*> boxes_;
    std::vector<NodeBox*> selected_boxes_;

    std::map<NodeBox*, ProfilingWidget*> profiling_;
    std::map<NodeBox*, std::vector<csapex::slim_signal::Connection>> profiling_connections_;

    int scalings_to_perform_;
    QTimer scalings_animation_timer_;

    QTimer scroll_animation_timer_;
    double scroll_offset_x_;
    double scroll_offset_y_;

    bool middle_mouse_dragging_;
    bool middle_mouse_panning_;
    QPointF middle_mouse_drag_start_;

    QDragMoveEvent* move_event_;

    std::unordered_map<UUID, NodeBox*, UUID::Hasher> box_map_;
    std::unordered_map<UUID, MovableGraphicsProxyWidget*, UUID::Hasher> proxy_map_;

    MessagePreviewWidget* preview_widget_;
};
}

#endif // GRAPH_VIEW_H
