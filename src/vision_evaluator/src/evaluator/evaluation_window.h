#ifndef EVALUATION_WINDOW_H
#define EVALUATION_WINDOW_H

/// SYSTEM
#include <QMainWindow>
#include <QTimer>

namespace Ui
{
class EvaluationWindow;
}

namespace vision_evaluator
{

/**
 * @brief The EvaluationWindow class provides the window for the evaluator program
 */
class EvaluationWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief EvaluationWindow
     * @param directory path to the initial directory for browsers
     * @param parent
     */
    explicit EvaluationWindow(const std::string& directory, QWidget* parent = 0);

    void closeEvent(QCloseEvent* event);

private Q_SLOTS:
    void updateMenu();
    void updateTitle();
    void updateLog();
    void hideLog();
    void kill();

public Q_SLOTS:
    void start();

private:
    Ui::EvaluationWindow* ui;

    QTimer timer;
    QTimer kill_timer;
};

} /// NAMESPACE

#endif // EVALUATION_WINDOW_H
