/*
 * qt_helper.hpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef QT_HELPER_HPP
#define QT_HELPER_HPP

/// SYSTEM
#include <QSlider>
#include <QBoxLayout>
#include <QSpinBox>
#include <QLabel>
#include <utils/qdouble_slider.h>
#include <QDoubleSpinBox>

class QtHelper
{
public:
    static QSlider* makeSlider(QBoxLayout* layout, const std::string& name, int def, int min, int max) {
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

    static QDoubleSlider* makeDoubleSlider(QBoxLayout* layout, const std::string& name, double def, double min, double max, double step_size) {
        QHBoxLayout* internal_layout = new QHBoxLayout;

        QDoubleSlider* slider = new QDoubleSlider(Qt::Horizontal, step_size);
        slider->setDoubleMinimum(min);
        slider->setDoubleMaximum(max);
        slider->setDoubleValue(def);
        slider->setMinimumWidth(100);

        QDoubleSpinBox* display = new QDoubleSpinBox;
        display->setMinimum(min);
        display->setMaximum(max);
        display->setValue(def);

        internal_layout->addWidget(new QLabel(name.c_str()));
        internal_layout->addWidget(slider);
        internal_layout->addWidget(display);

        layout->addLayout(internal_layout);

        QObject::connect(slider, SIGNAL(valueChanged(double)), display, SLOT(setValue(double)));
        QObject::connect(display, SIGNAL(valueChanged(double)), slider, SLOT(setDoubleValue(double)));

        return slider;
    }

    template<class Layout>
    static QWidget* wrapLayout(Layout *l)
    {
        QWidget *container = new QWidget;
        container->setLayout(l);
        return container;
    }

    static QHBoxLayout* wrap(const std::string& label, QWidget* widget) {

        QHBoxLayout* internal_layout = new QHBoxLayout;

        internal_layout->addWidget(new QLabel(label.c_str()));
        internal_layout->addWidget(widget);

        return internal_layout;
    }

    static void clearLayout(QLayout* layout) {
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
};

#endif // QT_HELPER_HPP
