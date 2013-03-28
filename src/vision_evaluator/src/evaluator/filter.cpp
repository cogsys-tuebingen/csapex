/// HEADER
#include "filter.h"

/// SYSTEM
#include <QLabel>
#include <QSpinBox>

using namespace vision_evaluator;


Filter::Filter()
    : name_("unnamed")
{
}

Filter::~Filter()
{
}

std::string Filter::getName()
{
    return name_;
}

void Filter::setName(const std::string &name)
{
    name_ = name;
}

QSlider* Filter::makeSlider(QBoxLayout* layout, const std::string& name, int def, int min, int max)
{
    QHBoxLayout* internal_layout = new QHBoxLayout;

    QSlider* slider = new QSlider(Qt::Horizontal);
    slider->setMinimum(min);
    slider->setMaximum(max);
    slider->setValue(def);
    slider->setMinimumWidth(100);

    QSpinBox* display = new QSpinBox;
    display->setMinimum(min);
    display->setMaximum(max);
    display->setValue(def);

    internal_layout->addWidget(new QLabel(name.c_str()));
    internal_layout->addWidget(slider);
    internal_layout->addWidget(display);

    layout->addLayout(internal_layout);

    QObject::connect(slider, SIGNAL(valueChanged(int)), display, SLOT(setValue(int)));
    QObject::connect(display, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));

    return slider;
}
