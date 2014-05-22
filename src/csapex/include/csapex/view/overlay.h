#ifndef OVERLAY_H
#define OVERLAY_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/connection.h>

/// SYSTEM
#include <map>
#include <QWidget>
#include <QPainter>

namespace csapex
{

class Overlay : public QWidget
{
    Q_OBJECT

public:
    Overlay(GraphPtr graph, CommandDispatcher* dispatcher, WidgetControllerPtr widget_ctrl, QWidget* parent = 0);

public Q_SLOTS:

    void tick();

    void blockMouse(bool);

public:
    bool mouseMoveEventHandler(bool drag, QMouseEvent * e);
    bool mousePressEventHandler(QMouseEvent * e);
    bool mouseReleaseEventHandler(QMouseEvent * e);

    bool keyPressEventHandler(QKeyEvent* e);
    bool keyReleaseEventHandler(QKeyEvent* e);

    void setSelectionRectangle(const QPoint& a, const QPoint& b);

    bool hasTempConnectionHandle() const;
    bool showContextMenu(const QPoint& pos);

protected:

    void paintEvent(QPaintEvent* event);

    QPen makeLinePen(const QPoint &from, const QPoint &to);
    QPen makeSelectedLinePen(const QPoint &from, const QPoint &to);

protected:
    CommandDispatcher* dispatcher_;
    WidgetControllerPtr widget_ctrl_;
    GraphPtr graph_;

    struct TempConnection {
        Connectable* from;
        QPoint to;
    };

    struct CurrentConnectionState {
        bool selected;
        bool highlighted;
        bool error;
        bool disabled;
        bool async;
        bool minimized_from;
        bool minimized_to;
        bool minimized;

        double r;
    };

    CurrentConnectionState ccs;

    std::vector<TempConnection> temp_;

    QPainter* painter;

    QTimer* repainter;

    int activity_marker_min_width_;
    int activity_marker_max_width_;
    int activity_marker_min_opacity_;
    int activity_marker_max_opacity_;

    int highlight_connection_id_;

    int connector_radius_;

    QPoint selection_a;
    QPoint selection_b;

    QPoint drag_connection_handle_;
    QPoint current_splicing_handle_;
    int drag_sub_section_;
    int drag_connection_;
    bool fulcrum_is_hovered_;

    bool mouse_blocked;
    bool splicing_requested;
    bool splicing;
};

}
#endif // OVERLAY_H
