/*
 * qt_helper.hpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef QT_HELPER_HPP
#define QT_HELPER_HPP

/// COMPONENT
#include <csapex/utility/qdouble_slider.h>
#include <csapex/utility/qwrapper.h>

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
    static QSpinBox* makeSpinBox(QBoxLayout *layout, const std::string &name, int def, int min, int max, int step_size = 1) ;

    static QSlider* makeSlider(QBoxLayout* layout, const std::string& name, int def, int min, int max, int step_size = 1);

    static QDoubleSlider* makeDoubleSlider(QBoxLayout* layout, const std::string& name, double def, double min, double max, double step_size);

    static QWidget* wrapLayout(QBoxLayout *l, QWidget *parent = 0);

    static QHBoxLayout* wrap(const std::string& label, QWidget* widget);

    static QHBoxLayout* wrap(const std::string& label, QLayout* layout);

    static void clearLayout(QLayout* layout);
};

#endif // QT_HELPER_HPP
