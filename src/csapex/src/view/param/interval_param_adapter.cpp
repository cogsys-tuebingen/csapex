/// HEADER
#include <csapex/view/param/interval_param_adapter.h>

/// PROJECT
#include <csapex/view/utility/qwrapper.h>
#include <csapex/view/node/parameter_context_menu.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/utility/assert.h>
#include <csapex/utility/type.h>
#include <csapex/command/update_parameter.h>
#include <csapex/view/widgets/doublespanslider.h>

/// SYSTEM
#include <QPointer>
#include <QBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QInputDialog>
#include <QApplication>
#include <iostream>
#include <qxt5/qxtspanslider.h>

using namespace csapex;


IntervalParameterAdapter::IntervalParameterAdapter(param::IntervalParameter::Ptr p)
    : ParameterAdapter(std::dynamic_pointer_cast<param::Parameter>(p)), interval_p_(p),
      internal_layout(new QHBoxLayout)
{

}

QWidget* IntervalParameterAdapter::setup(QBoxLayout* layout, const std::string& display_name)
{
    QLabel* label = new QLabel(QString::fromStdString(display_name));
    label->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(label, &QLabel::customContextMenuRequested,
                     [=](const QPoint& point){ customContextMenuRequested(label, point); });

    internal_layout->addWidget(label);

    if(interval_p_->is<std::pair<int, int> >()) {
        genericSetup<int, QxtSpanSlider, QWrapper::QSpinBoxExt>();
    } else if(interval_p_->is<std::pair<double, double> >()) {
        genericSetup<double, DoubleSpanSlider, QWrapper::QDoubleSpinBoxExt>();
    } else {
        layout->addWidget(new QLabel((display_name + "'s type is not yet implemented (range: " + type2name(p_->type()) + ")").c_str()));
    }

    for(int i = 0; i < internal_layout->count(); ++i) {
        QWidget* child = internal_layout->itemAt(i)->widget();
        child->setProperty("parameter", QVariant::fromValue(static_cast<void*>(static_cast<csapex::param::Parameter*>(p_.get()))));
    }

    layout->addLayout(internal_layout);

    return label;
}

namespace {
template <typename Slider>
struct MakeSlider
{
    static Slider* makeSlider(Qt::Orientation orientation, int) {
        return new Slider(orientation);
    }
};

template <>
struct MakeSlider<DoubleSpanSlider>
{
    static DoubleSpanSlider* makeSlider(Qt::Orientation orientation, double step) {
        return new DoubleSpanSlider(orientation, step);
    }
};


template <typename T, typename Slider, typename Spinbox>
struct WidgetConnector
{
    static void connect(IntervalParameterAdapter* parent,
                        QPointer<Slider>& slider,
                        QPointer<Spinbox>& displayLower,
                        QPointer<Spinbox>& displayUpper)
    {
        parent->connect(displayLower.data(), static_cast<void(Spinbox::*)(T)>(&Spinbox::valueChanged),
                        slider.data(), &Slider::setLowerValue);
        parent->connect(displayUpper.data(), static_cast<void(Spinbox::*)(T)>(&Spinbox::valueChanged),
                        slider.data(), &Slider::setUpperValue);

    }
};

template <>
struct WidgetConnector<double, DoubleSpanSlider, QWrapper::QDoubleSpinBoxExt>
{
    static void connect(IntervalParameterAdapter* parent,
                        QPointer<DoubleSpanSlider>& slider,
                        QPointer<QWrapper::QDoubleSpinBoxExt>& displayLower,
                        QPointer<QWrapper::QDoubleSpinBoxExt>& displayUpper)
    {
        parent->connect(displayLower.data(), static_cast<void(QWrapper::QDoubleSpinBoxExt::*)(double)>(&QWrapper::QDoubleSpinBoxExt::valueChanged),
                        slider.data(), &DoubleSpanSlider::setLowerDoubleValue);
        parent->connect(displayUpper.data(), static_cast<void(QWrapper::QDoubleSpinBoxExt::*)(double)>(&QWrapper::QDoubleSpinBoxExt::valueChanged),
                        slider.data(), &DoubleSpanSlider::setUpperDoubleValue);
    }
};

template <typename T, typename Slider>
struct Extractor
{
    static std::pair<T,T> makePair(Slider* slider) {
        return std::make_pair(slider->lowerValue(), slider->upperValue());
    }
};

template <>
struct Extractor<double, DoubleSpanSlider>
{
    static std::pair<double, double> makePair(DoubleSpanSlider* slider) {
        return std::make_pair(slider->lowerDoubleValue(), slider->upperDoubleValue());
    }
};
}

template <typename T, typename Slider, typename Spinbox>
void IntervalParameterAdapter::genericSetup()
{
    const std::pair<T,T>& v = interval_p_->as<std::pair<T,T> >();

    T min = interval_p_->min<T>();
    T max = interval_p_->max<T>();
    T step = interval_p_->step<T>();

    apex_assert_hard(min<=max);
    apex_assert_hard(step > 0);

    QPointer<Slider> slider = MakeSlider<Slider>::makeSlider(Qt::Horizontal, step);
    slider->setRange(min, max);
    slider->setSpan(v.first, v.second);
    slider->setSingleStep(step);

    QPointer<Spinbox> displayLower = new Spinbox;
    displayLower->setRange(min, max);
    displayLower->setValue(v.first);
    displayLower->setSingleStep(step);
    displayLower->setKeyboardTracking(false);

    QPointer<Spinbox> displayUpper = new Spinbox;
    displayUpper->setRange(min, max);
    displayUpper->setValue(v.second);
    displayUpper->setSingleStep(step);
    displayLower->setKeyboardTracking(false);


    internal_layout->addWidget(displayLower);
    internal_layout->addWidget(slider);

    internal_layout->addWidget(displayUpper);


    slider->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(slider.data(), &QLabel::customContextMenuRequested,
                     [=](const QPoint& point){ customContextMenuRequested(slider, point); });

    QObject::connect(slider.data(), &Slider::rangeChanged, displayLower.data(), &Spinbox::setRange);
    QObject::connect(slider.data(), &Slider::rangeChanged, displayUpper.data(), &Spinbox::setRange);
    QObject::connect(slider.data(), &Slider::lowerValueChanged, displayLower.data(), &Spinbox::setValue);
    QObject::connect(slider.data(), &Slider::upperValueChanged, displayUpper.data(), &Spinbox::setValue);

    WidgetConnector<T, Slider, Spinbox>::connect(this, slider, displayLower, displayUpper);

    auto cb = [this, slider]() {
        if(!interval_p_ || !slider) {
            return;
        }
        auto value = Extractor<T, Slider>::makePair(slider);
        command::UpdateParameter::Ptr update_parameter = std::make_shared<command::UpdateParameter>(AUUID(interval_p_->getUUID()), value);
        executeCommand(update_parameter);
    };

    connect(slider.data(), &Slider::lowerValueChanged, cb);
    connect(slider.data(), &Slider::upperValueChanged, cb);

    // model change -> ui
    connectInGuiThread(p_->parameter_changed, [this, slider, displayLower, displayUpper](param::Parameter*){
        if(!interval_p_ || !slider || !displayLower || !displayUpper) {
            return;
        }
        slider->blockSignals(true);
        displayLower->blockSignals(true);
        displayUpper->blockSignals(true);

        auto low = interval_p_->lower<T>();
        auto up = interval_p_->upper<T>();

        slider->setSpan(low, up);
        displayLower->setValue(low);
        displayUpper->setValue(up);

        slider->blockSignals(false);
        displayLower->blockSignals(false);
        displayUpper->blockSignals(false);
    });
    connectInGuiThread(interval_p_->step_changed, [this, slider, displayLower, displayUpper](param::Parameter*){
        if(!interval_p_ || !slider || !displayLower || !displayUpper) {
            return;
        }
        slider->blockSignals(true);
        displayLower->blockSignals(true);
        displayUpper->blockSignals(true);

        auto step = interval_p_->step<T>();

        slider->setSingleStep(step);
        displayLower->setSingleStep(step);
        displayUpper->setSingleStep(step);

        slider->blockSignals(false);
        displayLower->blockSignals(false);
        displayUpper->blockSignals(false);
    });

    // parameter scope changed -> update slider interval
    connectInGuiThread(p_->scope_changed, [this, slider, displayLower, displayUpper](param::Parameter*){
        if(!interval_p_ || !slider) {
            return;
        }
        slider->blockSignals(true);
        displayLower->blockSignals(true);
        displayUpper->blockSignals(true);

        auto min = interval_p_->min<T>();
        auto max = interval_p_->max<T>();
        slider->setRange(min, max);
        displayLower->setRange(min, max);
        displayUpper->setRange(min, max);

        slider->blockSignals(false);
        displayLower->blockSignals(false);
        displayUpper->blockSignals(false);
    });
}


void IntervalParameterAdapter::setupContextMenu(ParameterContextMenu *context_handler)
{
    context_handler->addAction(new QAction("reset to default", context_handler), [this](){
        if(interval_p_->is<std::pair<int, int>>()) {
            interval_p_->set(interval_p_->def<int>());
        } else if(interval_p_->is<std::pair<double, double>>()) {
            interval_p_->set(interval_p_->def<double>());
        }
    });

    context_handler->addAction(new QAction("set step size", context_handler), [this](){
        if(interval_p_->is<std::pair<int, int>>()) {
            int s = QInputDialog::getInt(QApplication::activeWindow(), "Step size", "Please enter the new step size",
                                         interval_p_->step<int>());
            interval_p_->setStep(s);
        } else if(interval_p_->is<std::pair<double, double>>()) {
            double s = QInputDialog::getDouble(QApplication::activeWindow(), "Step size", "Please enter the new step size",
                                               interval_p_->step<double>(),
                                               -1000., 1000.,
                                               8);
            interval_p_->setStep(s);
        }
    });
    context_handler->addAction(new QAction("set minimum", context_handler), [this](){
        if(interval_p_->is<std::pair<int, int>>()) {
            int s = QInputDialog::getInt(QApplication::activeWindow(), "Minimum", "Please enter the new minimum value",
                                         interval_p_->min<int>());
            interval_p_->setMin(s);
        } else if(interval_p_->is<std::pair<double, double>>()) {
            double s = QInputDialog::getDouble(QApplication::activeWindow(), "Minimum", "Please enter the new minimum value",
                                               interval_p_->min<double>(),
                                               -10000000., 10000000.,
                                               8);
            interval_p_->setMin(s);
        }
    });
    context_handler->addAction(new QAction("set maximum", context_handler), [this](){
        if(interval_p_->is<std::pair<int, int>>()) {
            int s = QInputDialog::getInt(QApplication::activeWindow(), "Maximum", "Please enter the new maximum value",
                                         interval_p_->max<int>());
            interval_p_->setMax(s);
        } else if(interval_p_->is<std::pair<double, double>>()) {
            double s = QInputDialog::getDouble(QApplication::activeWindow(), "Maximum", "Please enter the new maximum value",
                                               interval_p_->max<double>(),
                                               -10000000., 10000000.,
                                               8);
            interval_p_->setMax(s);
        }
    });
}
