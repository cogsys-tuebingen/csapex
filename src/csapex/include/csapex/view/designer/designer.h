#ifndef DESIGNER_H
#define DESIGNER_H

/// COMPONENT
#include <csapex/core/core_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/view/designer/designer_styleable.h>
#include <csapex/utility/uuid.h>
#include <csapex/utility/slim_signal.h>
#include <csapex/view/designer/designer_options.h>

/// SYSTEM
#include <QWidget>
#include <yaml-cpp/yaml.h>
#include <unordered_map>

/// FORWARD DECLARATIONS
namespace Ui
{
class Designer;
}

namespace csapex
{

class NodeFactory;

class Designer : public QWidget
{
    Q_OBJECT

    friend class DesignerIO;
    friend class DesignerOptions;

public:
    Designer(Settings& settings, NodeFactory& node_factory, NodeAdapterFactory &node_adapter_factory, GraphFacadePtr main_graph_facade, MinimapWidget* minimap, CommandDispatcher* dispatcher,
             DragIO& dragio, QWidget* parent = 0);
    virtual ~Designer();

    DesignerOptions *options();

    void setup();

    void setView(int x, int y);

    void addGraph(GraphFacadePtr graph);
    void removeGraph(GraphFacadePtr graph);

    GraphView* getVisibleGraphView() const;
    GraphView* getGraphView(const UUID& uuid) const;

    GraphFacade* getVisibleGraphFacade() const;
    DesignerScene* getVisibleDesignerScene() const;

    bool hasSelection() const;

    void saveSettings(YAML::Node& doc);
    void loadSettings(YAML::Node& doc);

    void saveView(Graph *graph, YAML::Node &e);
    void loadView(Graph* graph, YAML::Node& doc);

Q_SIGNALS:
    void selectionChanged();
    void helpRequest(NodeBox*);

public Q_SLOTS:
    void showGraph(UUID uuid);

    void closeView(int page);

    void addBox(NodeBox* box);
    void removeBox(NodeBox* box);

    void overwriteStyleSheet(QString& stylesheet);

    void updateMinimap();

    void refresh();
    void reset();

    std::vector<NodeBox*> getSelectedBoxes() const;
    void selectAll();
    void clearSelection();
    void deleteSelected();
    void copySelected();
    void paste();

private:
    void observe(GraphFacadePtr graph);
    void showGraph(GraphFacadePtr graph);

private:
    Ui::Designer* ui;
    DesignerStyleable style;

    DesignerOptions options_;

    DragIO& drag_io;
    MinimapWidget* minimap_;

    Settings& settings_;
    NodeFactory& node_factory_;
    NodeAdapterFactory& node_adapter_factory_;

    GraphFacadePtr root_graph_facade_;
    std::unordered_map<UUID, GraphFacadePtr, UUID::Hasher> graphs_;

    std::set<Graph*> visible_graphs_;
    std::map<Graph*, GraphView*> graph_views_;
    std::map<GraphView*, GraphFacade*> view_graphs_;

    std::map<UUID, YAML::Node> states_for_invisible_graphs_;

    CommandDispatcher* dispatcher_;

    bool space_;
    bool drag_;
    QPoint drag_start_pos_;

    bool is_init_;

    std::vector<csapex::slim_signal::ScopedConnection> connections_;
};

}
#endif // DESIGNER_H
