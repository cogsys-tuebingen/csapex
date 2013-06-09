/*
 * generic_manager.hpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef GENERIC_MANAGER_H
#define GENERIC_MANAGER_H

/// COMPONENT
#include "constructor.hpp"

/// PROJECT
#include <designer/selector_proxy.h>

/// SYSTEM
#include <boost/signals2.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/construct.hpp>
#include <pluginlib/class_loader.h>

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
        constructor.name = #class_name; \
        constructor.constructor = boost::lambda::new_ptr<class_name>(); \
        vision_evaluator::Manager manager;\
        manager.registerConstructor(constructor); \
    \
        SelectorProxy::ProxyConstructor c;\
        c.name = constructor.name;\
        c.constructor = boost::lambda::bind(boost::lambda::new_ptr<SelectorProxyImp<class_name> >(), boost::lambda::_1, (QWidget*) NULL); \
        SelectorProxy::registerProxy(c);\
    });\



template <class M, class C>
class PluginManagerImp
{
    template <class, class>
    friend class PluginManager;

protected:
    typedef C Constructor;
    typedef std::map<std::string, Constructor> Constructors;

    static PluginManagerImp<M,C>& instance(const std::string& full_name) {
        static PluginManagerImp<M,C> i(full_name);
        return i;
    }

protected:
    typedef pluginlib::ClassLoader<M> Loader;

    PluginManagerImp(const std::string& full_name)
        : loader_(new Loader("vision_evaluator", full_name)) {
    }
    PluginManagerImp(const PluginManagerImp& rhs);
    PluginManagerImp& operator = (const PluginManagerImp& rhs);

protected:
    virtual ~PluginManagerImp() {
    }

    void registerConstructor(Constructor constructor) {
        available_classes[constructor.getName()] = constructor;
    }

    void reload() {
        try {
            std::vector<std::string> classes = loader_->getDeclaredClasses();
            for(std::vector<std::string>::iterator c = classes.begin(); c != classes.end(); ++c) {
                std::cout << "load library for class " << *c << std::endl;
                loader_->loadLibraryForClass(*c);

                Constructor constructor;
                constructor.name = *c;
                constructor.constructor = boost::bind(&Loader::createUnmanagedInstance, loader_, *c);
                registerConstructor(constructor);
                std::cout << "loaded " << typeid(M).name() << " class " << *c << std::endl;
            }
        } catch(pluginlib::PluginlibException& ex) {
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
        plugins_loaded_ = true;
    }

protected:
    bool plugins_loaded_;
    Loader* loader_;

    Constructors available_classes;
};

template <class M, class C = DefaultConstructor<M> >
class PluginManager
{
protected:
    typedef PluginManagerImp<M, C> Parent;

public:
    typedef typename Parent::Constructor Constructor;
    typedef typename Parent::Constructors Constructors;

    PluginManager(const std::string& full_name)
        : instance(Parent::instance(full_name))
    {}

    virtual void registerConstructor(Constructor constructor) {
        instance.registerConstructor(constructor);
    }

    bool pluginsLoaded() const {
        return instance.plugins_loaded_;
    }

    void reload() {
        instance.reload();
    }

    const Constructors& availableClasses() const {
        return instance.available_classes;
    }
    const Constructor& availableClasses(unsigned index) const {
        typename Constructors::iterator it = instance.available_classes.begin();
        std::advance(it, index);
        return it->second;
    }
    const Constructor& availableClasses(const std::string& key) const {
        return instance.available_classes[key];
    }
    Constructor& availableClasses(const std::string& key) {
        return instance.available_classes[key];
    }

protected:
    Parent& instance;
};

#endif // GENERIC_MANAGER_H
