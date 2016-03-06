/// HEADER
#include <csapex/view/param/range_param_adapter.h>

/// PROJECT
#include <csapex/view/widgets/qint_slider.h>
#include <csapex/view/utility/qwrapper.h>
#include <csapex/view/utility/context_menu_handler.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/utility/assert.h>
#include <csapex/utility/type.h>

/// SYSTEM
#include <QPointer>
#include <QBoxLayout>
#include <QLabel>
#include <QApplication>
#include <iostream>

using namespace csapex;

namespace {

QIntSlider* makeIntSlider(QBoxLayout* layout, const std::string& name, int def, int min, int max, int step,
                          param::RangeParameterPtr range_param,
                          csapex::ContextMenuHandler *context_handler) {
    apex_assert_hard(min<=max);

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


    QHBoxLayout* internal_layout = new QHBoxLayout;

    QIntSlider* slider = new QIntSlider(Qt::Horizontal, step);
    slider->setIntMinimum(min);
    slider->setIntMaximum(max);
    slider->setIntValue(def);
    slider->setMinimumWidth(100);
    slider->setSingleStep(step);

    QWrapper::QSpinBoxExt* display = new QWrapper::QSpinBoxExt;
    display->setMinimum(min);
    display->setMaximum(max);
    display->setValue(def);
    display->setSingleStep(step);

    QLabel* label = new QLabel(name.c_str());
    if(context_handler) {
        label->setContextMenuPolicy(Qt::CustomContextMenu);
        context_handler->setParent(label);
        QObject::connect(label, SIGNAL(customContextMenuRequested(QPoint)), context_handler, SLOT(showContextMenu(QPoint)));
    }

    internal_layout->addWidget(label);
    internal_layout->addWidget(slider);
    internal_layout->addWidget(display);

    layout->addLayout(internal_layout);


    for(int i = 0; i < internal_layout->count(); ++i) {
        QWidget* child = internal_layout->itemAt(i)->widget();
        child->setProperty("parameter", QVariant::fromValue(static_cast<void*>(static_cast<csapex::param::Parameter*>(range_param.get()))));
    }

    QObject::connect(slider, SIGNAL(intValueChanged(int)),  display, SLOT(setValue(int)));
    QObject::connect(slider, SIGNAL(intRangeChanged(int,int)), display, SLOT(setRange(int,int)));
    QObject::connect(display, SIGNAL(valueChanged(int)), slider, SLOT(setIntValue(int)));


    return slider;
}


QDoubleSlider* makeDoubleSlider(QBoxLayout* layout, const std::string display_name,
                                param::RangeParameterPtr range_param,
                                csapex::ContextMenuHandler *context_handler)
{
    apex_assert_hard(range_param->min<double>()<=range_param->max<double>());

    QHBoxLayout* internal_layout = new QHBoxLayout;

    QDoubleSlider* slider = new QDoubleSlider(Qt::Horizontal, range_param->step<double>());
    slider->setDoubleMaximum(range_param->max<double>());
    slider->setDoubleMinimum(range_param->min<double>());
    slider->setDoubleValue(range_param->def<double>());
    slider->setMinimumWidth(100);

    // iterate until decimal value < e
    double e = 0.0001;
    int decimals = 0;
    double f = range_param->step<double>();
    while (true) {
        double decimal_val = f - (floor(f));
        if(decimal_val < e) {
            break;
        }
        f *= 10.0;
        ++decimals;
    }

    QWrapper::QDoubleSpinBoxExt* display = new  QWrapper::QDoubleSpinBoxExt;
    display->setDecimals(decimals);
    //    display->setDecimals(std::log10(1.0 / step_size) + 1);
    display->setMinimum(range_param->min<double>());
    display->setMaximum(range_param->max<double>());
    display->setValue(range_param->def<double>());
    display->setSingleStep(range_param->step<double>());

    QLabel* label = new QLabel(QString::fromStdString(display_name));
    if(context_handler) {
        label->setContextMenuPolicy(Qt::CustomContextMenu);
        context_handler->setParent(label);
        QObject::connect(label, SIGNAL(customContextMenuRequested(QPoint)), context_handler, SLOT(showContextMenu(QPoint)));
    }

    internal_layout->addWidget(label);
    internal_layout->addWidget(slider);
    internal_layout->addWidget(display);


    layout->addLayout(internal_layout);


    for(int i = 0; i < internal_layout->count(); ++i) {
        QWidget* child = internal_layout->itemAt(i)->widget();
        child->setProperty("parameter", QVariant::fromValue(static_cast<void*>(static_cast<csapex::param::Parameter*>(range_param.get()))));
    }

    QObject::connect(slider, SIGNAL(doubleValueChanged(double)), display, SLOT(setValue(double)));
    QObject::connect(slider, SIGNAL(doubleRangeChanged(double,double)), display, SLOT(setRange(double, double)));
    QObject::connect(display, SIGNAL(valueChanged(double)), slider, SLOT(setNearestDoubleValue(double)));

    return slider;
}


void assertGuiThread()
{
    apex_assert_hard(QThread::currentThread() == QApplication::instance()->thread());
}

void assertNotGuiThread()
{
    apex_assert_hard(QThread::currentThread() != QApplication::instance()->thread());
}


}


RangeParameterAdapter::RangeParameterAdapter(param::Proxy<param::RangeParameter>::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Proxy<param::Parameter>>(p)), range_p_(p)
{

}

void RangeParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    if(p_->is<int>()) {
        QPointer<QIntSlider> slider = makeIntSlider(layout, display_name ,
                                           range_p_->def<int>(), range_p_->min<int>(), range_p_->max<int>(), range_p_->step<int>(),
                                           range_p_,
                                           new ParameterContextMenu(p_));
        slider->setIntValue(p_->as<int>());

        // ui change -> model
        QObject::connect(slider.data(), &QIntSlider::intValueChanged, [this, slider](int value){
            assertNotGuiThread();
            if(!p_ || !slider) {
                return;
            }
            p_->set<int>(value);
        });


        // model change -> ui
        connectInGuiThread(p_->parameter_changed, [this, slider](){
            assertGuiThread();
            if(!p_ || !slider) {
                return;
            }
            slider->setIntValue(p_->as<int>());
        });

        // parameter scope changed -> update slider interval
        connectInGuiThread(p_->scope_changed, [this, slider]() {
            assertGuiThread();
            if(!p_ || !slider) {
                return;
            }
            slider->setIntRange(range_p_->min<int>(), range_p_->max<int>());
        });

    } else if(p_->is<double>()) {
        QPointer<QDoubleSlider> slider = makeDoubleSlider(layout, display_name , range_p_, new ParameterContextMenu(p_));
        slider->setDoubleValue(p_->as<double>());

        // ui change -> model
        QObject::connect(slider.data(), &QDoubleSlider::doubleValueChanged, [this, slider](double value){
            assertNotGuiThread();
            if(!p_ || !slider) {
                return;
            }
            p_->set<double>(value);
        });


        // model change -> ui
        connectInGuiThread(p_->parameter_changed, [this, slider]() {
            assertGuiThread();
            if(!p_ || !slider) {
                return;
            }
            slider->setDoubleValue(p_->as<double>());
        });

        // parameter scope changed -> update slider interval
        connectInGuiThread(p_->scope_changed, [this, slider]() {

            assertGuiThread();
            if(!p_ || !slider) {
                return;
            }
            slider->setDoubleRange(range_p_->min<double>(), range_p_->max<double>());
        });

    } else {
        layout->addWidget(new QLabel((display_name + "'s type is not yet implemented (range: " + type2name(p_->type()) + ")").c_str()));
    }
}
