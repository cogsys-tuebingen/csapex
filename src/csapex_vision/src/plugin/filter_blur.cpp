/// HEADER
#include "filter_blur.h"

/// PROJECT
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(csapex::FilterBlur, csapex::BoxedObject)

using namespace csapex;

FilterBlur::FilterBlur()
{
    state.blur = 0;
}

void FilterBlur::filter(cv::Mat& img, cv::Mat& mask)
{
    if(state.blur > 0) {
        cv::blur(img, img, cv::Size(state.blur, state.blur));
    }
}

void FilterBlur::insert(QBoxLayout* layout)
{
    slider = QtHelper::makeSlider(layout, "blur", 0, 0, 26);

    QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(update(int)));
}


void FilterBlur::update(int slot)
{
    state.blur = slot;

    Q_EMIT filter_changed();
}

Memento::Ptr FilterBlur::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void FilterBlur::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state = *m;

    slider->setValue(state.blur);

    Q_EMIT filter_changed();
}
