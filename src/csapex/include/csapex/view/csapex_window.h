#ifndef CSAPEX_WINDOW_H
#define CSAPEX_WINDOW_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/view/view_fwd.h>
#include <csapex/core/core_fwd.h>
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/utility/slim_signal.hpp>
#include <csapex/profiling/profiler.h>
#include <csapex/utility/utility_fwd.h>

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
class CSAPEX_QT_EXPORT CsApexWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief CsApexWindow
     * @param parent
     */
    explicit CsApexWindow(CsApexViewCore &core, QWidget* parent = 0);
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
    void showNotification(const Notification& notification);
    void updateNodeTypes();
    void updateSnippets();

    void updateDebugInfo();
    void updateUndoInfo();
    void updateNodeInfo();
    void updateThreadInfo();

    void about();
    void copyRight();
    void clearBlock();
    void resetActivity();
    void enableDebugProfiling(bool enabled);

    void loadTutorial(const QModelIndex& index);

Q_SIGNALS:
    void statusChanged(const QString& status);
    void showNotificationRequest(const Notification& notification);
    void closed();

private:
    void construct();
    void setupDesigner();


    void createPluginsMenu();
    void createTutorialsMenu();
    void loadStyleSheet();

    std::string getConfigFile();

private:
    CsApexViewCore& view_core_;
    CsApexCore& core_;
    CommandDispatcher* cmd_dispatcher_;
    GraphFacadePtr root_;
    Executor& executor_;

    std::shared_ptr<Profiler> profiler_;

    Ui::CsApexWindow* ui;

    Designer* designer_;
    MinimapWidget* minimap_;
    ActivityLegend* activity_legend_;
    ActivityTimeline* activity_timeline_;

    QTimer timer;

    bool init_;

    QFileSystemWatcher* style_sheet_watcher_;
    PluginLocatorPtr plugin_locator_;

    std::vector<csapex::slim_signal::ScopedConnection> connections_;
};

} /// NAMESPACE

#endif // CSAPEX_WINDOW_H
