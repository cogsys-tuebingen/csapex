#ifndef EVALUATION_WINDOW_H
#define EVALUATION_WINDOW_H

/// COMPONENT
#include <csapex/core_plugin.h>
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_plugin/plugin_manager.hpp>

/// SYSTEM
#include <QMainWindow>
#include <QTimer>

namespace Ui
{
class EvaluationWindow;
}

namespace csapex
{

/**
 * @brief The EvaluationWindow class provides the window for the evaluator program
 */
class CsApexWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief EvaluationWindow
     * @param parent
     */
    explicit CsApexWindow(QWidget* parent = 0);

    void showMenu();
    void closeEvent(QCloseEvent* event);
    void paintEvent(QPaintEvent * e);

    std::string getConfig() const;
    void setCurrentConfig(const std::string& filename);

private Q_SLOTS:
    void updateMenu();
    void updateTitle();
    void tick();
    void hideLog();
    void scrollDownLog();
    void init();

public Q_SLOTS:
    void save();
    void saveAs();
    void load();
    void reload();

    void start();
    void showStatusMessage(const std::string& msg);

Q_SIGNALS:
    void initialize();
    void configChanged();

private:
    void saveAs(const std::string& file);

private:
    Ui::EvaluationWindow* ui;

    std::string current_config_;

    Designer* designer_;
    PluginManager<CorePlugin> core_plugin_manager;

    QTimer timer;

    bool init_;
};

} /// NAMESPACE

#endif // EVALUATION_WINDOW_H
