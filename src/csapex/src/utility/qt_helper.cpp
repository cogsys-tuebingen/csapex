/// HEADER
#include <csapex/utility/qt_helper.hpp>

/// COMPONENT
#include <csapex/model/connector_in.h>
#include <csapex/manager/connection_type_manager.h>
#include <csapex/view/port.h>
#include <csapex/command/meta.h>

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

QSpinBox* QtHelper::makeSpinBox(QBoxLayout *layout, const std::string &name, int def, int min, int max) {
    QHBoxLayout *internal_layout = new QHBoxLayout;

    QSpinBox* spinner = new QSpinBox;
    spinner->setMinimum(min);
    spinner->setMaximum(max);
    spinner->setValue(def);

    internal_layout->addWidget(new QLabel(name.c_str()));
    internal_layout->addWidget(spinner);
    layout->addLayout(internal_layout);

    return spinner;
}

QSlider* QtHelper::makeSlider(QBoxLayout* layout, const std::string& name, int def, int min, int max) {
    QHBoxLayout* internal_layout = new QHBoxLayout;

    /// TODO: Make connector independent of node
    //Port* cin = new Port(new QtConnectable("foo", "int"));

    QSlider* slider = new QSlider(Qt::Horizontal);
    slider->setMinimum(min);
    slider->setMaximum(max);
    slider->setValue(def);
    slider->setMinimumWidth(100);

    QWrapper::QSpinBoxExt* display = new QWrapper::QSpinBoxExt;
    display->setMinimum(min);
    display->setMaximum(max);
    display->setValue(def);

    //internal_layout->addWidget(cin);
    internal_layout->addWidget(new QLabel(name.c_str()));
    internal_layout->addWidget(slider);
    internal_layout->addWidget(display);

    layout->addLayout(internal_layout);

    QObject::connect(slider, SIGNAL(valueChanged(int)),     display, SLOT(setValue(int)));
    QObject::connect(slider, SIGNAL(rangeChanged(int,int)), display, SLOT(setRange(int,int)));
    QObject::connect(display, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));


    return slider;
}

QDoubleSlider* QtHelper::makeDoubleSlider(QBoxLayout* layout, const std::string& name, double def, double min, double max, double step_size) {
    QHBoxLayout* internal_layout = new QHBoxLayout;

    QDoubleSlider* slider = new QDoubleSlider(Qt::Horizontal, step_size);
    slider->setDoubleMinimum(min);
    slider->setDoubleMaximum(max);
    slider->setDoubleValue(def);
    slider->setMinimumWidth(100);

    QWrapper::QDoubleSpinBoxExt* display = new  QWrapper::QDoubleSpinBoxExt;
    display->setDecimals(std::log10(1.0 / step_size));
    display->setMinimum(min);
    display->setMaximum(max);
    display->setValue(def);

    internal_layout->addWidget(new QLabel(name.c_str()));
    internal_layout->addWidget(slider);
    internal_layout->addWidget(display);

    layout->addLayout(internal_layout);

    QObject::connect(slider, SIGNAL(valueChanged(double)), display, SLOT(setValue(double)));
    QObject::connect(slider, SIGNAL(rangeChanged(double,double)), display, SLOT(setRange(double, double)));
    QObject::connect(display, SIGNAL(valueChanged(double)), slider, SLOT(setDoubleValue(double)));

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
