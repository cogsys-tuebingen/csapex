/// HEADER
#include <csapex/view/param/range_param_adapter.h>

/// PROJECT
#include <csapex/view/widgets/qint_slider.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/utility/assert.h>
#include <csapex/utility/type.h>
#include <csapex/command/update_parameter.h>

/// SYSTEM
#include <QPointer>
#include <QBoxLayout>
#include <QLabel>
#include <QApplication>
#include <iostream>

using namespace csapex;

RangeParameterAdapter::RangeParameterAdapter(param::RangeParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), range_p_(p),
      internal_layout(new QHBoxLayout)
{

}

namespace {
void setDecimals(QSpinBox*, int)
{
    // do nothing
}

void setDecimals(QDoubleSpinBox* box, double step)
{
    int decimals = 0;
    double v = step;
    while(v < 1.0) {
        v *= 10;
        ++decimals;
    }
    box->setDecimals(decimals);
}
}

void RangeParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QLabel* label = new QLabel(QString::fromStdString(display_name));
    if(context_handler) {
        label->setContextMenuPolicy(Qt::CustomContextMenu);
        context_handler->setParent(label);
        QObject::connect(label, SIGNAL(customContextMenuRequested(QPoint)), context_handler, SLOT(showContextMenu(QPoint)));
    }

    internal_layout->addWidget(label);

    if(range_p_->is<int>()) {
        genericSetup<int, QIntSlider, QWrapper::QSpinBoxExt>();
    } else if(range_p_->is<double>()) {
        genericSetup<double, QDoubleSlider, QWrapper::QDoubleSpinBoxExt>();
    } else {
        layout->addWidget(new QLabel((display_name + "'s type is not yet implemented (range: " + type2name(p_->type()) + ")").c_str()));
    }

    for(int i = 0; i < internal_layout->count(); ++i) {
        QWidget* child = internal_layout->itemAt(i)->widget();
        child->setProperty("parameter", QVariant::fromValue(static_cast<void*>(static_cast<csapex::param::Parameter*>(p_.get()))));
    }

    layout->addLayout(internal_layout);
}

template <typename T, typename Slider, typename Spinbox>
void RangeParameterAdapter::genericSetup()
{
    T min = range_p_->min<T>();
    T max = range_p_->max<T>();
    T step = range_p_->step<T>();

    T def = range_p_->def<T>();

    if(((def - min) / step) * step != (def - min)) {
        std::cerr << "default " << def << " is not a multiple of minimum " << min << " with a step size of " << step << std::endl;
        def = min;
        std::cerr << "set default to " << def << std::endl;
    }

    if(((max - min) / step) * step != (max - min)) {
        std::cerr << "maximum " << max << " is not a multiple of minimum " << min << " with a step size of " << step << std::endl;
        max = ((max - min) / step) * step + min;
        std::cerr << "set maximum to " << max << std::endl;
    }

    QPointer<Slider> slider = new Slider(Qt::Horizontal, step);
    slider->setScaledMinimum(min);
    slider->setScaledMaximum(max);
    slider->setScaledValue(def);
    slider->setMinimumWidth(100);
    slider->setSingleStep(step);

    QPointer<Spinbox> display = new Spinbox;
    display->setMinimum(min);
    display->setMaximum(max);
    display->setValue(def);
    display->setSingleStep(step);
    setDecimals(display.data(), step);

    internal_layout->addWidget(slider);
    internal_layout->addWidget(display);

    QObject::connect(slider.data(), &Slider::scaledValueChanged,
                     display.data(), &Spinbox::setValue);
    QObject::connect(slider.data(), &Slider::scaledRangeChanged,
                     display.data(), &Spinbox::setRange);
    QObject::connect(display.data(), static_cast<void(Spinbox::*)(T)>(&Spinbox::valueChanged),
                     slider.data(), &Slider::setScaledValue);

    slider->setScaledValue(p_->as<T>());

    // ui change -> model
    QObject::connect(slider.data(), &Slider::scaledValueChanged, [this, slider](T value){
        if(!p_ || !slider) {
            return;
        }

        command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(p_->getUUID()), value);
        executeCommand(update_parameter);
    });


    // model change -> ui
    connectInGuiThread(p_->parameter_changed, [this, slider, display](){
        if(!p_ || !slider || !display) {
            return;
        }
        auto v = p_->as<T>();
        slider->blockSignals(true);
        display->blockSignals(true);

        slider->setScaledValue(v);
        display->setValue(v);

        display->blockSignals(false);
        slider->blockSignals(false);
    });

    // parameter scope changed -> update slider interval
    connectInGuiThread(p_->scope_changed, [this, slider, display]() {
        if(!p_ || !slider || !display) {
            return;
        }
        slider->blockSignals(true);
        display->blockSignals(true);

        slider->setScaledRange(range_p_->min<T>(), range_p_->max<T>());
        display->setValue(slider->scaledValue());

        display->blockSignals(false);
        slider->blockSignals(false);
    });
}
