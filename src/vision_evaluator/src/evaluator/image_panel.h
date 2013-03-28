#ifndef IMAGE_PANEL_H
#define IMAGE_PANEL_H

/// COMPONENT
#include "panel.h"
#include "viewer.h"
#include "filter_manager.h"

/// SYSTEM
#include <QObject>
#include <QModelIndex>

/// FORWARD DECLARATIONS
namespace Ui
{
class ImagePanel;
}

class QFileSystemModel;

namespace vision_evaluator
{

/**
 * @brief The ImagePanel class displays an image and connects it to a viewer
 */
class ImagePanel : public Panel
{
    Q_OBJECT

public:
    /**
     * @brief ImagePanel
     * @param parent
     */
    explicit ImagePanel(QWidget* parent = 0);

    /**
     * @brief ~ImagePanel
     */
    virtual ~ImagePanel();

    void quit();
    void wait();

public Q_SLOTS:
    void set_root(const std::string& path) const;
    void set_root(QModelIndex index) const;
    void select(QModelIndex index);
    void go_up();
    void go_location();
    void minimize_splitter();
    void restore_splitter();
    void update();
    void set_fps(int fps);
    void nextImage();
    void setOneShotModeOn();
    void setOneShotModeOff();

Q_SIGNALS:
    void handle(const std::string&);
    void updateRequest();

private:
    class SleeperThread : public QThread
    {
    public:
        static void msleep(unsigned long msecs) {
            QThread::msleep(msecs);
        }
    };


private:
    Ui::ImagePanel* ui;

    FilterManager filter_manager;
    QFileSystemModel* model;

    Viewer* viewer;

    int splitter_width_;
};

} /// NAMESPACE

#endif // IMAGE_PANEL_H
