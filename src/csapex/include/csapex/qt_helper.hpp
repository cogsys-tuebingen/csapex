/*
 * qt_helper.hpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef QT_HELPER_HPP
#define QT_HELPER_HPP

/// COMPONENT
#include <csapex/qdouble_slider.h>
#include <csapex/qwrapper.h>

/// SYSTEM
#include <QSlider>
#include <QBoxLayout>
#include <QSpinBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QThread>
#include <cmath>
#include <boost/function.hpp>

namespace qt_helper {

struct Call : public QObject
{
    Q_OBJECT

    typedef boost::function<void()> CB;

public:
    Call(CB cb)
        : cb_(cb)
    {}

public Q_SLOTS:
    void call() {
        cb_();
    }

private:
    CB cb_;
};

struct QSleepThread : public QThread {
    static void sleep(unsigned long t) {
        QThread::sleep(t);
    }
    static void msleep(unsigned long t) {
        QThread::msleep(t);
    }
    static void usleep(unsigned long t) {
        QThread::usleep(t);
    }
};

}

class QtHelper
{
public:
    static QSpinBox* makeSpinBox(QBoxLayout *layout, const std::string &name, int def, int min, int max) {
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

    static QSlider* makeSlider(QBoxLayout* layout, const std::string& name, int def, int min, int max) {
        QHBoxLayout* internal_layout = new QHBoxLayout;

        QSlider* slider = new QSlider(Qt::Horizontal);
        slider->setMinimum(min);
        slider->setMaximum(max);
        slider->setValue(def);
        slider->setMinimumWidth(100);

        QWrapper::QSpinBoxExt* display = new QWrapper::QSpinBoxExt;
        display->setMinimum(min);
        display->setMaximum(max);
        display->setValue(def);

        internal_layout->addWidget(new QLabel(name.c_str()));
        internal_layout->addWidget(slider);
        internal_layout->addWidget(display);

        layout->addLayout(internal_layout);

        QObject::connect(slider, SIGNAL(valueChanged(int)),     display, SLOT(setValue(int)));
        QObject::connect(slider, SIGNAL(rangeChanged(int,int)), display, SLOT(setRange(int,int)));
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

    static QWidget* wrapLayout(QBoxLayout *l, QWidget *parent = 0)
    {
        QWidget *container = new QWidget(parent);
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
