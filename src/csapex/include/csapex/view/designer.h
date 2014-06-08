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
             DesignerView *view, DesignerScene* scene,  QWidget* parent = 0);
    virtual ~Designer();

    void setup();

    void setView(int x, int y);

    DesignerView* getDesignerView();

    bool isGridEnabled() const;
    bool isGridLockEnabled() const;

    bool hasSelection() const;

Q_SIGNALS:
    void selectionChanged();
    void gridEnabled(bool);
    void schematicsEnabled(bool);
    void gridLockEnabled(bool);

public Q_SLOTS:
    void addBox(NodeBox* box);
    void removeBox(NodeBox* box);
    void stateChangedEvent();

    void overwriteStyleSheet(QString& stylesheet);

    void enableGrid(bool);
    void enableSchematics(bool);
    void lockToGrid(bool);

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
