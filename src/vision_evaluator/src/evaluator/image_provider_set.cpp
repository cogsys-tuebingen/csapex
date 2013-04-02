/// HEADER
#include "image_provider_set.h"

/// SYSTEM
#include <QHBoxLayout>

using namespace vision_evaluator;

ImageProviderSet::ImageProviderSet()
    : playing_(true), current_frame(0), next_frame(-1)
{
}

ImageProviderSet::~ImageProviderSet()
{
}

void ImageProviderSet::update_gui(QFrame* additional_holder)
{
    QHBoxLayout* layout = new QHBoxLayout(additional_holder);

    slider_ = new QSlider(Qt::Horizontal);
    slider_->setMaximum(frames_);

    play_pause_ = new QPushButton(tr("Play / Pause"));
    play_pause_->setCheckable(true);
    play_pause_->setChecked(playing_);

    layout->addWidget(play_pause_);
    layout->addWidget(slider_);

    additional_holder->setLayout(layout);

    QObject::connect(slider_, SIGNAL(sliderMoved(int)), this, SLOT(showFrame()));
    QObject::connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(showFrame()));
    QObject::connect(play_pause_, SIGNAL(toggled(bool)), this, SLOT(setPlaying(bool)));
}

void ImageProviderSet::next()
{
    if(playing_ || next_frame != -1) {
        reallyNext();
        next_frame = -1;

    } else {
        provide(last_frame_);
    }
}

void ImageProviderSet::provide(cv::Mat frame)
{
    cv::Mat clone;
    frame.copyTo(clone);
    Q_EMIT new_image(clone, cv::Mat());
}

void ImageProviderSet::setPlaying(bool playing)
{
    playing_ = playing;
    if(play_pause_->isChecked() != playing) {
        play_pause_->setChecked(playing);
    }
}

void ImageProviderSet::showFrame()
{
    int frame = slider_->value();

    if(frame == current_frame) {
        return;
    }
    std::cout << "skip to frame " << frame << " / " << frames_ << std::endl;

    next_frame = frame;
}

