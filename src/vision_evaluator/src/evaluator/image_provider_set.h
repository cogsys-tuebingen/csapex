#ifndef IMAGE_PROVIDER_SET_H
#define IMAGE_PROVIDER_SET_H

/// COMPONENT
#include "image_provider.h"

/// SYSTEM
#include <QPushButton>
#include <QSlider>

namespace vision_evaluator
{

class ImageProviderSet : public ImageProvider
{
    Q_OBJECT

protected:
    ImageProviderSet();
    virtual ~ImageProviderSet();

public:
    virtual void insert(QBoxLayout* layout);
    virtual void update_gui(QFrame* additional_holder);
    virtual void next(cv::Mat& img, cv::Mat& mask);

protected Q_SLOTS:
    void showFrame();
    void setPlaying(bool playing);

protected: // abstract
    virtual void reallyNext(cv::Mat& img, cv::Mat& mask) = 0;

protected:
    cv::Mat last_frame_;

    QPushButton* play_pause_;
    QSlider* slider_;

    bool playing_;

    double fps_;
    double frames_;
    int current_frame;
    int next_frame;
};

} /// NAMESPACE

#endif // IMAGE_PROVIDER_SET_H
