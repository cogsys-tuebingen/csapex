#ifndef IMAGE_COMBINER_PANEL_H
#define IMAGE_COMBINER_PANEL_H

/// COMPONENT
#include "panel.h"
#include "image_combiner_manager.h"

/// SYSTEM
#include <QWidget>
#include <opencv2/opencv.hpp>

/// FORWARD DECLARATIONS
namespace Ui
{
class ImageCombinerPanel;
}

namespace vision_evaluator
{

class ImageCombinerPanel : public Panel
{
    Q_OBJECT

public:
    explicit ImageCombinerPanel(QWidget* parent = 0);

    bool eventFilter(QObject* target, QEvent* event);

public Q_SLOTS:
    void input_1(const cv::Mat img, const cv::Mat mask);
    void input_2(const cv::Mat img, const cv::Mat mask);

    void quit();
    void wait();

Q_SIGNALS:
    void handle(cv::Mat,cv::Mat,cv::Mat,cv::Mat);
    void nextImageRequest();
    void combinerInstalled();
    void combinerDeinstalled();


private:
    void handle();

private:
    Ui::ImageCombinerPanel* ui;

    ImageCombinerManager combiner_manager;

    bool has_1;
    bool has_2;

    cv::Mat img1;
    cv::Mat mask1;
    cv::Mat img2;
    cv::Mat mask2;
};

} /// NAMESPACE

#endif // IMAGE_COMBINER_PANEL_H
