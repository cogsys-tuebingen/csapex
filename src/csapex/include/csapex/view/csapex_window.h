#ifndef CSAPEX_WINDOW_H
#define CSAPEX_WINDOW_H

/// COMPONENT
#include <csapex/view/view_fwd.h>
#include <csapex/core/core_fwd.h>
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <QMainWindow>
#include <QTimer>
#include <QFileSystemWatcher>
#include <QBoxLayout>

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
    explicit CsApexWindow(CsApexCore &core, CommandDispatcher *cmd_dispatcher,
                          GraphFacadePtr graph_facade, Executor &executor,
                          Designer *designer, MinimapWidget* minimap, ActivityLegend *legend, ActivityTimeline* timeline,
                          PluginLocatorPtr locator, QWidget* parent = 0);
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
    void showHelp(NodeBox* box);
    void showHowToInstall();

    void updateSelectionActions();
    void updateClipboardActions();

    void updatePluginIgnored(const QObject *&action);

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

    void updateDebugInfo();
    void updateUndoInfo();
    void updateNodeInfo();

    void about();
    void copyRight();
    void clearBlock();
    void resetActivity();

Q_SIGNALS:
    void statusChanged(const QString& status);
    void closed();

private:
    void construct();
    void setupDesigner();

    void createPluginsMenu();
    void loadStyleSheet();

    std::string getConfigFile();

private:
    CsApexCore& core_;
    CommandDispatcher* cmd_dispatcher_;
    GraphFacadePtr root_;
    Executor& executor_;

    Ui::CsApexWindow* ui;

    Designer* designer_;
    MinimapWidget* minimap_;
    ActivityLegend* activity_legend_;
    ActivityTimeline* activity_timeline_;

    QTimer timer;

    bool init_;

    QFileSystemWatcher* style_sheet_watcher_;
    PluginLocatorPtr plugin_locator_;

    std::vector<csapex::slim_signal::Connection> connections_;
};

} /// NAMESPACE

#endif // CSAPEX_WINDOW_H
