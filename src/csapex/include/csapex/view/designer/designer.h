#ifndef DESIGNER_H
#define DESIGNER_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/core/core_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/utility/slim_signal.h>
#include <csapex/view/csapex_view_core.h>
#include <csapex/profiling/profilable.h>
#include <csapex/utility/notification.h>
#include <csapex/model/observer.h>

/// SYSTEM
#include <QWidget>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <deque>

/// FORWARD DECLARATIONS
namespace Ui
{
class Designer;
}

class QFrame;
class QParallelAnimationGroup;

namespace csapex
{

class NodeFactory;
class NotificationWidget;

class CSAPEX_QT_EXPORT Designer : public QWidget, public Profilable, public Observer
{
    Q_OBJECT

    friend class DesignerIO;
    friend class DesignerOptions;

public:
    Designer(CsApexViewCore& view_core, QWidget* parent = 0);
    virtual ~Designer();

    DesignerOptions *options();
    MinimapWidget* getMinimap();

    void setup();

    void setView(int x, int y);

    void addGraph(GraphFacadePtr graph);
    void removeGraph(GraphFacade *graph);

    GraphView* getVisibleGraphView() const;
    GraphView* getGraphView(const AUUID &uuid) const;

    GraphFacade* getVisibleGraphFacade() const;
    DesignerScene* getVisibleDesignerScene() const;

    NodeAdapterFactory* getNodeAdapterFactory() const;

    bool hasSelection() const;

    void saveSettings(YAML::Node& doc);
    void loadSettings(YAML::Node& doc);

    void saveView(SubgraphNode *graph, YAML::Node &e);
    void loadView(SubgraphNode *graph, YAML::Node& doc);


    virtual void useProfiler(std::shared_ptr<Profiler> profiler) override;

Q_SIGNALS:
    void selectionChanged();
    void helpRequest(NodeBox*);

public Q_SLOTS:
    void showGraph(UUID uuid);
    void showNodeDialog();
    void showNodeSearchDialog();

    void closeView(int page);

    void addBox(NodeBox* box);
    void removeBox(NodeBox* box);

    void focusOnNode(const AUUID& id);

    void overwriteStyleSheet(QString& stylesheet);

    void updateMinimap();

    void showNotification(const Notification& notification);
    void removeNotification(const Notification& notification);

    void refresh();
    void reset();
    void reinitialize();

    std::vector<NodeBox*> getSelectedBoxes() const;
    void selectAll();
    void clearSelection();
    void deleteSelected();
    void copySelected();
    void paste();
    void groupSelected();
    void ungroupSelected();

private:
    void resizeEvent(QResizeEvent* re);

    void observeGraph(GraphFacadePtr graph);
    void showGraph(GraphFacadePtr graph);

private:
    Ui::Designer* ui;

    DesignerOptions options_;

    MinimapWidget* minimap_;

    CsApexCore& core_;
    CsApexViewCore& view_core_;

    std::unordered_map<UUID, GraphFacadePtr, UUID::Hasher> graphs_;

    std::set<SubgraphNode*> visible_graphs_;
    std::map<SubgraphNode*, GraphView*> graph_views_;
    std::map<AUUID, GraphView*> auuid_views_;
    std::map<GraphView*, GraphFacade*> view_graphs_;
    std::map<SubgraphNode*, std::vector<slim_signal::ScopedConnection>> graph_connections_;
    std::map<GraphView*, std::vector<slim_signal::ScopedConnection>> view_connections_;

    std::map<UUID, YAML::Node> states_for_invisible_graphs_;

    bool space_;
    bool drag_;
    QPoint drag_start_pos_;

    bool is_init_;

    QParallelAnimationGroup* notification_animation_;
    std::unordered_map<UUID, NotificationWidget*, UUID::Hasher> named_notifications_;
    std::deque<NotificationWidget*> sorted_notifications_;
};

}
#endif // DESIGNER_H
