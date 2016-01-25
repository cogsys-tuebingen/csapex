#ifndef GRAPH_VIEW_H
#define GRAPH_VIEW_H

/// COMPONENT
#include <csapex/view/view_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/core/core_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/view/designer/designer_styleable.h>
#include <csapex/utility/slim_signal.hpp>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <QGraphicsView>
#include <map>
#include <QTimer>
#include <unordered_map>

namespace csapex
{

class NodeFactory;

class GraphView : public QGraphicsView
{
    Q_OBJECT

    Q_PROPERTY(QColor lineColor         READ lineColor         WRITE setLineColor)
    Q_PROPERTY(QColor lineColorError    READ lineColorError    WRITE setLineColorError)
    Q_PROPERTY(QColor lineColorBlocked  READ lineColorBlocked  WRITE setLineColorBlocked)
    Q_PROPERTY(QColor lineColorDisabled READ lineColorDisabled WRITE setLineColorDisabled)
    Q_PROPERTY(int lineWidth            READ lineWidth         WRITE setLineWidth)

    void setLineColor(const QColor& c) { style_->setLineColor(c); }
    void setLineColorError(const QColor& c) { style_->setLineColorError(c); }
    void setLineColorBlocked(const QColor& c) { style_->setLineColorBlocked(c); }
    void setLineColorDisabled(const QColor& c) { style_->setLineColorDisabled(c);}
    void setLineWidth(int width) { style_->setLineWidth(width); }

    QColor lineColor() const { return style_->lineColor(); }
    QColor lineColorError() const { return style_->lineColorError(); }
    QColor lineColorBlocked() const { return style_->lineColorBlocked(); }
    QColor lineColorDisabled() const { return style_->lineColorDisabled();}
    int lineWidth() const { return style_->lineWidth(); }

public:
    GraphView(DesignerScene* scene, csapex::GraphFacadePtr graph,
              Settings& settings, NodeFactory& node_factory, NodeAdapterFactory& node_adapter_factory,
              CommandDispatcher *dispatcher, DragIO& dragio,
              DesignerStyleable* style, Designer *parent);
    ~GraphView();

    DesignerScene* designerScene();
    std::vector<NodeBox*> boxes();
    std::vector<NodeBox*> getSelectedBoxes();
    CommandPtr deleteSelected();

    NodeBox* getBox(const UUID& node_id);
    MovableGraphicsProxyWidget* getProxy(const UUID& node_id);

    GraphFacade* getGraphFacade() const;

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

    void paintEvent(QPaintEvent* e);

    void nodeAdded(NodeWorkerPtr node_worker);
    void nodeRemoved(NodeHandlePtr node_handle);

    void startPlacingBox(const std::string& type, NodeStatePtr state, const QPoint &offset = QPoint(0,0));

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
    void showBoxDialog();

    void addBox(NodeBox* box);
    void removeBox(NodeBox* box);

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

    void updateSelection();
    void selectAll();

    void copySelected();
    void paste();

    void showPreview(Port* port);
    void stopPreview();

private:
    void flipBox();
    void minimizeBox(bool mini);
    void deleteBox();
    void groupBox();
    void usePrivateThreadFor();
    void switchToThread(int group_id);
    void createNewThreadGroupFor();

    void showProfiling(bool visible);

private:
    Designer* parent_;
    DesignerScene* scene_;
    DesignerStyleable* style_;

    Settings& settings_;
    NodeFactory& node_factory_;
    NodeAdapterFactory& node_adapter_factory_;

    GraphFacadePtr graph_facade_;
    CommandDispatcher* dispatcher_;
    DragIO& drag_io_;

    PortPanel* relayed_inputs_widget_;
    QGraphicsProxyWidget* relayed_inputs_widget_proxy_;
    PortPanel* relayed_outputs_widget_;
    QGraphicsProxyWidget* relayed_outputs_widget_proxy_;

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
    std::unordered_map<UUID, Port*, UUID::Hasher> port_map_;

    MessagePreviewWidget* preview_widget_;
};
}

#endif // GRAPH_VIEW_H
