/*
 * qt_helper.hpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef QT_HELPER_HPP
#define QT_HELPER_HPP

 /// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/view/widgets/qdouble_slider.h>
#include <csapex/view/widgets/qint_slider.h>
#include <csapex/view/utility/qwrapper.h>
#include <csapex/view/utility/context_menu_handler.h>
#include <csapex/param/param_fwd.h>

/// SYSTEM
#include <QBoxLayout>
#include <QThread>
#include <cmath>

namespace qt_helper {

struct CSAPEX_QT_EXPORT QSleepThread {
    static void msleep(unsigned long t);
};

}

class CSAPEX_QT_EXPORT QtHelper
{
public:
    static QWidget* wrapLayout(QBoxLayout *l, QWidget *parent = 0);

    static QHBoxLayout* wrap(const std::string& label, QWidget* widget,
                             csapex::ContextMenuHandler *context_handler = nullptr,
                             csapex::param::Parameter* p = nullptr);

    static QHBoxLayout* wrap(const std::string& label, QLayout* layout,
                             csapex::ContextMenuHandler *context_handler = nullptr,
                             csapex::param::Parameter* p = nullptr);

    static void clearLayout(QLayout* layout);
};

#endif // QT_HELPER_HPP
