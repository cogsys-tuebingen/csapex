#ifndef DESIGNER_H
#define DESIGNER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

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
    Designer(Settings& settings, GraphPtr graph, CommandDispatcher* dispatcher, WidgetControllerPtr widget_ctrl,
             DesignerView *view, DesignerScene* scene, MinimapWidget* minimap, QWidget* parent = 0);
    virtual ~Designer();

    void setup();

    void setView(int x, int y);

    DesignerView* getDesignerView();

    bool isGridEnabled() const;
    bool isSchematicsEnabled() const;
    bool isGraphComponentsEnabled() const;
    bool isThreadsEnabled() const;
    bool isMinimapEnabled() const;
    bool areSignalConnectionsVisible() const;
    bool areMessageConnectionsVisibile() const;

    bool hasSelection() const;

Q_SIGNALS:
    void selectionChanged();
    void gridEnabled(bool);
    void minimapEnabled(bool);
    void signalsEnabled(bool);
    void messagesEnabled(bool);
    void schematicsEnabled(bool);
    void graphComponentsEnabled(bool);
    void threadsEnabled(bool);
    void helpRequest(NodeBox*);

public Q_SLOTS:
    void addBox(NodeBox* box);
    void removeBox(NodeBox* box);
    void stateChangedEvent();

    void overwriteStyleSheet(QString& stylesheet);

    void enableGrid(bool);
    void enableSchematics(bool);
    void displayGraphComponents(bool);
    void displayThreads(bool);
    void displayMinimap(bool);
    void displaySignalConnections(bool);
    void displayMessageConnections(bool);

    void refresh();
    void reset();

    std::vector<NodeBox*> getSelectedBoxes() const;
    void selectAll();
    void clearSelection();
    void deleteSelected();

private:
    Ui::Designer* ui;
    DesignerView* designer_view_;
    DesignerScene* designer_scene_;
    MinimapWidget* minimap_;

    Settings& settings_;
    GraphPtr graph_;
    CommandDispatcher* dispatcher_;
    WidgetControllerPtr widget_ctrl_;

    bool space_;
    bool drag_;
    QPoint drag_start_pos_;

    bool is_init_;
};

}
#endif // DESIGNER_H
