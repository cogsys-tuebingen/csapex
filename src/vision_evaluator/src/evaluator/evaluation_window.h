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

class Designer;

/**
 * @brief The EvaluationWindow class provides the window for the evaluator program
 */
class EvaluationWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief EvaluationWindow
     * @param parent
     */
    explicit EvaluationWindow(QWidget* parent = 0);

    void showMenu();
    void closeEvent(QCloseEvent* event);

    Designer* getDesigner();

private Q_SLOTS:
    void updateMenu();
    void updateTitle();
    void updateLog();
    void hideLog();
    void scrollDownLog();

public Q_SLOTS:
    void start();

private:
    Ui::EvaluationWindow* ui;

    QTimer timer;
};

} /// NAMESPACE

#endif // EVALUATION_WINDOW_H
