/*
 * generic_manager.hpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef GENERIC_MANAGER_H
#define GENERIC_MANAGER_H

/// PROJECT
#include <designer/selector_proxy.h>
#include <utils/constructor.hpp>

/// SYSTEM
#include <boost/signals2.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/construct.hpp>

#define STATIC_INIT(prefix, name, code)\
    namespace vision_evaluator { \
    class _____##prefix##name##_registrator { \
        static _____##prefix##name##_registrator instance; \
        _____##prefix##name##_registrator () {\
            { code } \
        }\
    };\
    _____##prefix##name##_registrator _____##prefix##name##_registrator::instance;\
    }


#define REGISTER_GENERIC(Manager, class_name)\
    STATIC_INIT(Manager, class_name, { \
        std::cout << "register filter instance " << #class_name << std::endl; \
        Manager::Constructor constructor; \
        constructor.setName(#class_name); \
        constructor.setConstructor(boost::lambda::new_ptr<class_name>()); \
        vision_evaluator::Manager manager;\
        manager.registerConstructor(constructor); \
    \
        SelectorProxy::ProxyConstructor c;\
        c.setName(#class_name);\
        c.setConstructor(boost::lambda::bind(boost::lambda::new_ptr<SelectorProxyImp<class_name> >(), boost::lambda::_1, (QWidget*) NULL)); \
        SelectorProxy::registerProxy(c);\
    });\


#endif // GENERIC_MANAGER_H
