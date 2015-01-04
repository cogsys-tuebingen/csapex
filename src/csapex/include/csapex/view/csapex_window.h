#ifndef CSAPEX_WINDOW_H
#define CSAPEX_WINDOW_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/core/csapex_core.h>

/// SYSTEM
#include <QMainWindow>
#include <QTimer>
#include <QFileSystemWatcher>
#include <QBoxLayout>

namespace Ui
{
class CsApexWindow;
}

namespace csapex
{

/**
 * @brief The CsApexWindow class provides the window for the evaluator program
 */
class CsApexWindow : public QMainWindow, public CsApexCore::Listener
{
    Q_OBJECT

public:
    /**
     * @brief CsApexWindow
     * @param parent
     */
    explicit CsApexWindow(CsApexCore &core, CommandDispatcher *cmd_dispatcher, WidgetControllerPtr widget_ctrl,
                          GraphWorkerPtr graph, Designer *designer, ActivityLegend *legend, ActivityTimeline* timeline,
                          PluginLocator* locator, QWidget* parent = 0);
    virtual ~CsApexWindow();

    void closeEvent(QCloseEvent* event);

    void resetSignal();

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

    void start();
    void showStatusMessage(const std::string& msg);
    void reloadBoxMenues();

    void saveSettings(YAML::Node& doc);
    void loadSettings(YAML::Node& doc);

    void saveView(YAML::Node &e);
    void loadView(YAML::Node& doc);

    void updateDebugInfo();
    void updateUndoInfo();
    void updateNodeInfo();

    void about();
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

    Ui::CsApexWindow* ui;

    Designer* designer_;
    ActivityLegend* activity_legend_;
    ActivityTimeline* activity_timeline_;

    QTimer timer;

    bool init_;

    QFileSystemWatcher* style_sheet_watcher_;
    PluginLocator* plugin_locator_;
};

} /// NAMESPACE

#endif // CSAPEX_WINDOW_H
