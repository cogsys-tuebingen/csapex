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
#include <csapex/utility/qint_slider.h>
#include <csapex/utility/qwrapper.h>
#include <csapex/csapex_fwd.h>
#include <csapex/utility/context_menu_handler.h>

/// PROJECT
#include <utils_qt/qxtspanslider.h>
#include <utils_qt/qxtdoublespanslider.h>

/// SYSTEM
#include <QSlider>
#include <QBoxLayout>
#include <QSpinBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QThread>
#include <cmath>

namespace qt_helper {

struct QSleepThread : public QThread {
    static void sleep(unsigned long t);
    static void msleep(unsigned long t);
    static void usleep(unsigned long t);
};

}

class QtHelper
{
public:
    static QWidget* wrapLayout(QBoxLayout *l, QWidget *parent = 0);

    static QHBoxLayout* wrap(const std::string& label, QWidget* widget,
                             csapex::ContextMenuHandler *context_handler = 0);

    static QHBoxLayout* wrap(const std::string& label, QLayout* layout,
                             csapex::ContextMenuHandler *context_handler = 0);

    static void clearLayout(QLayout* layout);
};

#endif // QT_HELPER_HPP
