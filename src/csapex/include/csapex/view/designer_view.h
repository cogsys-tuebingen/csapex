#ifndef DESIGNER_VIEW_H
#define DESIGNER_VIEW_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QGraphicsView>
#include <map>
#include <QTimer>

namespace csapex
{

class DesignerView : public QGraphicsView
{
    Q_OBJECT

    Q_PROPERTY(QColor inputColor READ inputColor WRITE setInputColor)
    Q_PROPERTY(QColor outputColor READ outputColor WRITE setOutputColor)

public:
    DesignerView(DesignerScene* scene, csapex::GraphPtr graph,
                 Settings& settings, ThreadPool& thread_pool, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl, DragIO& dragio, QWidget* parent = 0);
    ~DesignerView();

    DesignerScene* designerScene();

    CommandPtr deleteSelected();

    void keyPressEvent(QKeyEvent* e);
    void keyReleaseEvent(QKeyEvent* e);

    void wheelEvent(QWheelEvent* we);

    void mouseMoveEvent(QMouseEvent* me);
    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);
    void dragLeaveEvent(QDragLeaveEvent * e);

    QColor inputColor() const
    {
        return input_color_;
    }
    QColor outputColor() const
    {
        return output_color_;
    }

    void setInputColor(const QColor& c)
    {
        input_color_ = c;
    }
    void setOutputColor(const QColor& c)
    {
        output_color_ = c;
    }


Q_SIGNALS:
    void selectionChanged();

public Q_SLOTS:
    void showBoxDialog();
    void addBoxEvent(NodeBox* box);
    void removeBoxEvent(NodeBox* box);
    void renameBox(NodeBox* box);

    void movedBoxes(double dx, double dy);

    void overwriteStyleSheet(QString& stylesheet);
    void updateBoxInformation();

    void contextMenuEvent(QContextMenuEvent* e);
    void showContextMenuGlobal(const QPoint& pos);
    void showContextMenuEditBox(NodeBox* box, const QPoint& pos);
    void showContextMenuAddNode(const QPoint& global_pos);

    void profile(NodeBox* box);
    void stopProfiling(NodeBox* box);

    void reset();
    void resetZoom();
    void zoomIn();
    void zoomOut();
    void zoom(double factor);
    void animateZoom();

    void updateSelection();
    void selectAll();

private:
    void deleteBox(NodeBox* box);
    void createNewThreadGroupFor(NodeWorker* worker);

private:
    DesignerScene* scene_;

    Settings& settings_;
    ThreadPool& thread_pool_;

    GraphPtr graph_;
    CommandDispatcher* dispatcher_;
    WidgetControllerPtr widget_ctrl_;
    DragIO& drag_io_;

    QColor input_color_;
    QColor output_color_;

    std::vector<NodeBox*> boxes_;
    std::map<NodeBox*, ProfilingWidget*> profiling_;

    int scalings_to_perform_;
    QTimer scalings_animation_timer_;
};
}

#endif // DESIGNER_VIEW_H
