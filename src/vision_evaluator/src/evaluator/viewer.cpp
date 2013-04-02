/// HEADER
#include "viewer.h"

/// COMPONENT
#include "image_provider.h"

/// SYSTEM
#include <iostream>
#include <QLayout>
#include <QMetaType>

using namespace vision_evaluator;

Viewer::Viewer(QFrame* additional_holder)
    : ticker_(NULL), additional_holder_(additional_holder), provider_(NULL), fps(10), one_shot_mode(false), requests(1)
{
    ticker_ = new QTimer(this);
    QObject::connect(ticker_, SIGNAL(timeout()), this, SLOT(iteration()));

//    queue->viewer = this;
}

Viewer::~Viewer()
{
    delete ticker_;
}

void Viewer::clearLayout()
{
    QLayout* layout = additional_holder_->layout();
    QLayoutItem* item;
    QLayout* sublayout;
    QWidget* widget;
    while(layout && (item = layout->takeAt(0))) {
        if((sublayout = item->layout()) != 0) {
            /* do the same for sublayout*/
        } else if((widget = item->widget()) != 0) {
            widget->hide();
            delete widget;
        } else {
            delete item;
        }
    }

    delete layout;
}

void Viewer::setOneShotMode(bool mode)
{
    one_shot_mode = mode;

    if(mode) {
        requests = 1;
    }
}

void Viewer::update_gui()
{
    if(provider_) {
        clearLayout();
        provider_->update_gui(additional_holder_);

        Q_EMIT gui_updated();
    }
}

void Viewer::setProvider(QSharedPointer<ImageProvider> provider)
{
    provider_ = provider;

    QObject::connect(&*provider_, SIGNAL(new_image(cv::Mat, cv::Mat)), this, SIGNAL(image_provided(cv::Mat, cv::Mat)), Qt::DirectConnection);

    Q_EMIT update_request();
}

void Viewer::provideNextImage()
{
    requests++;
}

void Viewer::handle(const std::string& path)
{
    std::cout << "click on " << path << std::endl;

    last_path_ = path;

    ticker_->stop();

    QSharedPointer<ImageProvider> provider(ImageProvider::create(path));

    if(provider == NULL) {
        std::cerr << "no provider for type: " << path << std::endl;
        return;
    }
    setProvider(provider);
}

void Viewer::repeat()
{
    if(!last_path_.empty()) {
        handle(last_path_);
    }
}

void Viewer::tick()
{
    ticker_->start(1000.0 /*ms / s*/ / fps /*frames / s*/);
}

void Viewer::set_fps(int fps)
{
    ticker_->stop();

    this->fps = fps;
    tick();
}

void Viewer::iteration()
{
    if(one_shot_mode && requests <= 0) {
        requests = 0;
        return;
    }

    if(!provider_->hasNext()) {
//        WARN("called iteration on finished provider.");
        return;
    }

    //while(provider_->hasNext()) {
    provider_->next();

    requests--;

//        int sleep_time = provider_->sleepTime();
//        boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_time));
    // }
}
