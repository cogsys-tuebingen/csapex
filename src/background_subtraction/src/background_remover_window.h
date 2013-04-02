#ifndef BACKGROUND_REMOVER_WINDOW_H
#define BACKGROUND_REMOVER_WINDOW_H

/// COMPONENT
#include "background_remover_node.h"

/// PROJECT
#include "background_subtraction/GlobalConfig.h"

/// SYSTEM
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>

/// FORWARD DECLARATION
namespace Ui
{
class BackgroundRemoverWindow;
}

namespace background_subtraction
{

/**
 * @brief The BackgroundRemoverWindow class is a QT Window that wraps the BackgroundRemoverNode
 */
class BackgroundRemoverWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief BackgroundRemoverWindow
     * @param nh ROS NodeHandle
     * @param parent
     */
    explicit BackgroundRemoverWindow(ros::NodeHandle& nh, QWidget* parent = 0);

    /**
     * @brief ~BackgroundRemoverNode
     */
    virtual ~BackgroundRemoverWindow();

public Q_SLOTS:
    void updateImage();
    void updateUi();

    void changeAlgo(int i);
    void setThreshold(int i);
    void setOpen(int i);
    void setClose(int i);

    void shutdown();

private:
    Ui::BackgroundRemoverWindow* ui;

    QMutex img_mutex;
    BackgroundRemoverNode node;

    QGraphicsScene* lt;
    QGraphicsScene* rt;
    QGraphicsScene* lb;
    QGraphicsScene* t;
    QGraphicsScene* rb;
};

}

#endif // BACKGROUND_REMOVER_WINDOW_H
