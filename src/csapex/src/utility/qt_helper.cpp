/// HEADER
#include <csapex/utility/qt_helper.hpp>

/// COMPONENT
#include <csapex/model/connector_in.h>
#include <csapex/manager/connection_type_manager.h>
#include <csapex/view/port.h>
#include <csapex/command/meta.h>

using namespace qt_helper;
using namespace csapex;

class QtConnectable : public ConnectorIn
{
public:
    QtConnectable(const UUID& uuid, const std::string& type)
        : Connectable(uuid), ConnectorIn(uuid)
    {
        setType(ConnectionTypeManager::createMessage(type));
    }
};


void QSleepThread::sleep(unsigned long t) {
    currentThread()->sleep(t);
}
void QSleepThread::msleep(unsigned long t) {
    currentThread()->msleep(t);
}
void QSleepThread::usleep(unsigned long t) {
    currentThread()->usleep(t);
}

QSpinBox* QtHelper::makeSpinBox(QBoxLayout *layout, const std::string &name, int def, int min, int max, int step_size)
{
    assert(min<=max);

    QHBoxLayout *internal_layout = new QHBoxLayout;

    QSpinBox* spinner = new QSpinBox;
    spinner->setMinimum(min);
    spinner->setMaximum(max);
    spinner->setValue(def);
    spinner->setSingleStep(step_size);

    internal_layout->addWidget(new QLabel(name.c_str()));
    internal_layout->addWidget(spinner);
    layout->addLayout(internal_layout);

    return spinner;
}

QSlider* QtHelper::makeSlider(QBoxLayout* layout, const std::string& name, int def, int min, int max, int step)
{
    assert(min<=max);

    QHBoxLayout* internal_layout = new QHBoxLayout;

    QSlider* slider = new QSlider(Qt::Horizontal);
    slider->setMinimum(min);
    slider->setMaximum(max);
    slider->setValue(def);
    slider->setMinimumWidth(100);
    slider->setSingleStep(step);

    QWrapper::QSpinBoxExt* display = new QWrapper::QSpinBoxExt;
    display->setMinimum(min);
    display->setMaximum(max);
    display->setValue(def);
    display->setSingleStep(step);

    internal_layout->addWidget(new QLabel(name.c_str()));
    internal_layout->addWidget(slider);
    internal_layout->addWidget(display);

    layout->addLayout(internal_layout);

    QObject::connect(slider, SIGNAL(valueChanged(int)),     display, SLOT(setValue(int)));
    QObject::connect(slider, SIGNAL(rangeChanged(int,int)), display, SLOT(setRange(int,int)));
    QObject::connect(display, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));


    return slider;
}

QDoubleSlider* QtHelper::makeDoubleSlider(QBoxLayout* layout, const std::string& name, double def, double min, double max, double step_size)
{
    assert(min<=max);

    QHBoxLayout* internal_layout = new QHBoxLayout;

    QDoubleSlider* slider = new QDoubleSlider(Qt::Horizontal, step_size);
    slider->setDoubleMaximum(max);
    slider->setDoubleMinimum(min);
    slider->setDoubleValue(def);
    slider->setMinimumWidth(100);

    // iterate until decimal value < e
    double e = 0.0001;
    int decimals = 0;
    double f = step_size;
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
    display->setMinimum(min);
    display->setMaximum(max);
    display->setValue(def);
    display->setSingleStep(step_size);

    internal_layout->addWidget(new QLabel(name.c_str()));
    internal_layout->addWidget(slider);
    internal_layout->addWidget(display);

    layout->addLayout(internal_layout);

    QObject::connect(slider, SIGNAL(valueChanged(double)), display, SLOT(setValue(double)));
    QObject::connect(slider, SIGNAL(rangeChanged(double,double)), display, SLOT(setRange(double, double)));
    //problem: rueckkopplung -> slider ändert sich -> box update -> box ändert sich -> slider update
    QObject::connect(display, SIGNAL(valueChanged(double)), slider, SLOT(setNearestDoubleValue(double)));

    return slider;
}



QxtSpanSlider* QtHelper::makeSpanSlider(QBoxLayout* layout, const std::string& name, int lower, int upper, int min, int max)
{
    assert(min<=max);

    QHBoxLayout* internal_layout = new QHBoxLayout;

    QxtSpanSlider* slider = new QxtSpanSlider(Qt::Horizontal);
    slider->setMinimum(min);
    slider->setMaximum(max);
    slider->setUpperValue(upper);
    slider->setLowerValue(lower);

    QWrapper::QSpinBoxExt* displayLower = new QWrapper::QSpinBoxExt;
    displayLower->setMinimum(min);
    displayLower->setMaximum(max);
    displayLower->setValue(lower);

    QWrapper::QSpinBoxExt* displayUpper = new QWrapper::QSpinBoxExt;
    displayUpper->setMinimum(min);
    displayUpper->setMaximum(max);
    displayUpper->setValue(upper);

    internal_layout->addWidget(new QLabel(name.c_str()));
    internal_layout->addWidget(displayLower);
    internal_layout->addWidget(slider);
    internal_layout->addWidget(displayUpper);

    layout->addLayout(internal_layout);

    QObject::connect(slider,        SIGNAL(rangeChanged(int,int)),  displayUpper,   SLOT(setRange(int,int)));
    QObject::connect(slider,        SIGNAL(lowerValueChanged(int)), displayLower,   SLOT(setValue(int)));
    QObject::connect(slider,        SIGNAL(upperValueChanged(int)), displayUpper,   SLOT(setValue(int)));
    QObject::connect(displayLower,  SIGNAL(valueChanged(int)),      slider,         SLOT(setLowerValue(int)));
    QObject::connect(displayUpper,  SIGNAL(valueChanged(int)),      slider,         SLOT(setUpperValue(int)));

    return slider;
}


QxtDoubleSpanSlider* QtHelper::makeDoubleSpanSlider(QBoxLayout *layout, const std::string &name, double lower, double upper, double min, double max, double step_size)
{
    assert(min<=max);

    QHBoxLayout* internal_layout = new QHBoxLayout;

    QxtDoubleSpanSlider* slider = new QxtDoubleSpanSlider(Qt::Horizontal, step_size);
    slider->setDoubleMinimum(min);
    slider->setDoubleMaximum(max);
    slider->setUpperDoubleValue(upper);
    slider->setLowerDoubleValue(lower);

    QWrapper::QDoubleSpinBoxExt* displayLower = new QWrapper::QDoubleSpinBoxExt;
    displayLower->setRange(min, max);
    displayLower->setValue(lower);

    QWrapper::QDoubleSpinBoxExt* displayUpper = new QWrapper::QDoubleSpinBoxExt;
    displayUpper->setRange(min, max);
    displayUpper->setValue(upper);

    internal_layout->addWidget(new QLabel(name.c_str()));
    internal_layout->addWidget(displayLower);
    internal_layout->addWidget(slider);
    internal_layout->addWidget(displayUpper);

    layout->addLayout(internal_layout);

    QObject::connect(slider,        SIGNAL(rangeChanged(double,double)),  displayUpper,   SLOT(setRange(double,double)));
    QObject::connect(slider,        SIGNAL(lowerValueChanged(double)), displayLower,   SLOT(setValue(double)));
    QObject::connect(slider,        SIGNAL(upperValueChanged(double)), displayUpper,   SLOT(setValue(double)));
    QObject::connect(displayLower,  SIGNAL(valueChanged(double)),      slider,         SLOT(setLowerDoubleValue(double)));
    QObject::connect(displayUpper,  SIGNAL(valueChanged(double)),      slider,         SLOT(setUpperDoubleValue(double)));

    return slider;
}
QWidget* QtHelper::wrapLayout(QBoxLayout *l, QWidget *parent)
{
    QWidget *container = new QWidget(parent);
    container->setLayout(l);
    return container;
}

QHBoxLayout* QtHelper::wrap(const std::string& label, QWidget* widget) {

    QHBoxLayout* internal_layout = new QHBoxLayout;

    internal_layout->addWidget(new QLabel(label.c_str()));
    internal_layout->addWidget(widget);

    return internal_layout;
}

QHBoxLayout* QtHelper::wrap(const std::string& label, QLayout* layout) {

    QHBoxLayout* internal_layout = new QHBoxLayout;

    internal_layout->addWidget(new QLabel(label.c_str()));
    internal_layout->addLayout(layout);

    return internal_layout;
}

void QtHelper::clearLayout(QLayout* layout) {
    QLayoutItem* item;
    while((item = layout->takeAt(0)) != NULL) {
        if(item->layout()) {
            clearLayout(item->layout());
        }
        if(item->widget()) {
            delete item->widget();
        }
        delete item;
    }

}
