#ifndef CSAPEX_WINDOW_H
#define CSAPEX_WINDOW_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QMainWindow>
#include <QTimer>
#include <QFileSystemWatcher>
#include <QBoxLayout>
#include <boost/signals2/connection.hpp>

namespace Ui
{
class CsApexWindow;
}

namespace YAML
{
class Node;
}

namespace csapex
{

/**
 * @brief The CsApexWindow class provides the window for the evaluator program
 */
class CsApexWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief CsApexWindow
     * @param parent
     */
    explicit CsApexWindow(CsApexCore &core, CommandDispatcher *cmd_dispatcher, WidgetControllerPtr widget_ctrl,
                          GraphWorkerPtr graph_worker, GraphPtr graph, Executor &executor, Designer *designer, MinimapWidget* minimap, ActivityLegend *legend, ActivityTimeline* timeline,
                          PluginLocator* locator, QWidget* parent = 0);
    virtual ~CsApexWindow();

    void closeEvent(QCloseEvent* event);

    void setupTimeline();

private Q_SLOTS:
    void updateMenu();
    void updateTitle();
    void tick();
    void init();
    void reloadStyleSheet(const QString& path);
    void loadStyleSheet(const QString& path);
    void updateDeleteAction();
    void showHelp(NodeBox* box);
    void showHowToInstall();

    void updatePluginIgnored(const QObject *&action);
    void reloadPlugin(const QObject *&action);
    void updatePluginAutoReload(bool autoreload);

public Q_SLOTS:
    void save();
    void saveAs();
    void saveAsCopy();
    void load();
    void reload();
    void reset();
    void clear();
    void undo();
    void redo();

    void makeScreenshot();

    void start();
    void showStatusMessage(const std::string& msg);
    void updateNodeTypes();

    void saveSettings(YAML::Node& doc);
    void loadSettings(YAML::Node& doc);

    void saveView(YAML::Node &e);
    void loadView(YAML::Node& doc);

    void updateDebugInfo();
    void updateUndoInfo();
    void updateNodeInfo();

    void about();
    void copyRight();
    void clearBlock();

Q_SIGNALS:
    void statusChanged(const QString& status);

private:
    void construct();
    void createPluginsMenu();
    void loadStyleSheet();

    std::string getConfigFile();

private:
    CsApexCore& core_;
    CommandDispatcher* cmd_dispatcher_;
    WidgetControllerPtr widget_ctrl_;
    GraphWorkerPtr graph_worker_;
    GraphPtr graph_;
    Executor& executor_;

    Ui::CsApexWindow* ui;

    Designer* designer_;
    MinimapWidget* minimap_;
    ActivityLegend* activity_legend_;
    ActivityTimeline* activity_timeline_;

    QTimer timer;

    bool init_;

    QFileSystemWatcher* style_sheet_watcher_;
    PluginLocator* plugin_locator_;

    std::vector<boost::signals2::connection> connections_;
};

} /// NAMESPACE

#endif // CSAPEX_WINDOW_H
