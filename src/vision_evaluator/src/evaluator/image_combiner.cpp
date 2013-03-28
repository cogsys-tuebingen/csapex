/// HEADER
#include "image_combiner.h"

REGISTER_META_PLUGIN(ImageCombiner)

using namespace vision_evaluator;

ImageCombiner::ImageCombiner(const std::string& label)
    : Plugin(label)
{
}

PluginBase::PluginPtr ImageCombiner::createMetaInstance()
{
    return PluginPtr(new ImageCombiner("ImageCombiners"));
}

PluginBase::PluginPtr ImageCombiner::metaInstance()
{
    return ImageCombiner::createMetaInstance();
}

cv::Mat ImageCombiner::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(active.get() == NULL) {
        return img1;
    } else {
        return active->combine(img1, mask1, img2, mask2);
    }
}

void ImageCombiner::setQueue(PluginQueue* queue)
{
    Plugin::setQueue(queue);

    if(active != NULL) {
        active->setQueue(queue);
    }
}

void ImageCombiner::update()
{
    int i = 0;
    std::vector<ImageCombiner::Constructor>&constructors = instance().constructors;
    for(std::vector<QRadioButton*>::iterator it = buttons.begin(); it != buttons.end(); ++it) {
        QRadioButton* rb = *it;

        if(rb->isChecked()) {
            active = constructors[i](CONSTRUCTOR_INSTANTIATE);
            active->init_gui(NULL);
            active->setQueue(queue_);

            Q_EMIT combinerInstalled();

            return;
        }

        ++i;
    }

    Q_EMIT combinerDeinstalled();
}

void ImageCombiner::mousePressEvent(QMouseEvent* event)
{
    if(active != NULL) {
        active->mousePressEvent(event);
    }
}

void ImageCombiner::mouseMoveEvent(QMouseEvent* event)
{
    if(active != NULL) {
        active->mouseMoveEvent(event);
    }
}

void ImageCombiner::wheelEvent(QWheelEvent* event)
{
    if(active != NULL) {
        active->wheelEvent(event);
    }
}

void ImageCombiner::keyEvent(QKeyEvent* event)
{
    if(active != NULL) {
        active->keyEvent(event);
    }
}

void ImageCombiner::insert(QBoxLayout* layout)
{
    std::vector<ImageCombiner::Constructor>&constructors = instance().constructors;
    for(std::vector<ImageCombiner::Constructor>::iterator it = constructors.begin(); it != constructors.end(); ++it) {
        ImageCombiner::TypePtr combiner = (*it)(CONSTRUCTOR_META);

        QRadioButton* btn = new QRadioButton(combiner->getName().c_str());
        layout->addWidget(btn);
        buttons.push_back(btn);

        QObject::connect(btn, SIGNAL(toggled(bool)), this, SLOT(update()));
    }
}
