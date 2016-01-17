#ifndef DESIGNER_H
#define DESIGNER_H

/// COMPONENT
#include <csapex/core/core_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/view/designer/designer_styleable.h>

/// SYSTEM
#include <QWidget>
#include <QTreeWidget>

/// FORWARD DECLARATIONS
namespace Ui
{
class Designer;
}

namespace csapex
{

class Designer : public QWidget
{
    Q_OBJECT

    friend class DesignerIO;

public:
    Designer(Settings& settings, GraphFacadePtr main_graph_facade, MinimapWidget* minimap, CommandDispatcher* dispatcher, WidgetControllerPtr widget_ctrl,
             DragIO& dragio, QWidget* parent = 0);
    virtual ~Designer();

    void setup();

    void setView(int x, int y);

    GraphView* getGraphView();

    bool isGridEnabled() const;
    bool isSchematicsEnabled() const;
    bool isGraphComponentsEnabled() const;
    bool isThreadsEnabled() const;
    bool isMinimapEnabled() const;
    bool areSignalConnectionsVisible() const;
    bool areMessageConnectionsVisibile() const;
    bool isDebug() const;

    bool hasSelection() const;

Q_SIGNALS:
    void selectionChanged();
    void gridEnabled(bool);
    void minimapEnabled(bool);
    void signalsEnabled(bool);
    void messagesEnabled(bool);
    void debugEnabled(bool);
    void schematicsEnabled(bool);
    void graphComponentsEnabled(bool);
    void threadsEnabled(bool);
    void helpRequest(NodeBox*);

public Q_SLOTS:
    void addBox(NodeBox* box);
    void removeBox(NodeBox* box);

    void overwriteStyleSheet(QString& stylesheet);

    void enableGrid(bool);
    void enableSchematics(bool);
    void displayGraphComponents(bool);
    void displayThreads(bool);
    void displayMinimap(bool);
    void displaySignalConnections(bool);
    void displayMessageConnections(bool);
    void enableDebug(bool);

    void refresh();
    void reset();

    std::vector<NodeBox*> getSelectedBoxes() const;
    void selectAll();
    void clearSelection();
    void deleteSelected();
    void copySelected();
    void paste();

private:
    Ui::Designer* ui;
    DesignerStyleable style;

    DragIO& drag_io;
    MinimapWidget* minimap_;

    GraphView* main_graph_view_;
    DesignerScene* designer_scene_;

    Settings& settings_;
    GraphFacadePtr main_graph_facade_;
    CommandDispatcher* dispatcher_;
    WidgetControllerPtr widget_ctrl_;

    bool space_;
    bool drag_;
    QPoint drag_start_pos_;

    bool is_init_;
};

}
#endif // DESIGNER_H
