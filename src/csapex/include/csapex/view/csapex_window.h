#ifndef EVALUATION_WINDOW_H
#define EVALUATION_WINDOW_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/core/csapex_core.h>

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
    explicit CsApexWindow(CsApexCore &core, QWidget* parent = 0);

    void showMenu();
    void closeEvent(QCloseEvent* event);
    void paintEvent(QPaintEvent * e);

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
    void saveAsCopy();
    void load();
    void reload();
    void reset();

    void start();
    void showStatusMessage(const std::string& msg);

    void saveSettings(YAML::Emitter& e);
    void loadSettings(YAML::Node& doc);

Q_SIGNALS:
    void initialize();


private:
    CsApexCore& core;

    Ui::EvaluationWindow* ui;

    Designer* designer_;

    QTimer timer;

    bool init_;
};

} /// NAMESPACE

#endif // EVALUATION_WINDOW_H
