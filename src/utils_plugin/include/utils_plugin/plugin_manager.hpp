#ifndef PLUGIN_MANAGER_HPP
#define PLUGIN_MANAGER_HPP

/// COMPONENT
//#include <csapex/boxed_object.h>
//#include "selector_proxy.h"
#include <utils_plugin/constructor.hpp>

/// SYSTEM
#include <boost/signals2.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/construct.hpp>
#include <pluginlib/class_loader.h>

namespace plugin_manager {
template <class C>
struct InstallConstructor
{
    template <class M, class L>
    static void installConstructor(M* instance, L* loader, const std::string& name) {}
};
}

template <class M, class C>
class PluginManagerImp
{
    template <class, class>
    friend class PluginManager;

public:
    typedef pluginlib::ClassLoader<M> Loader;

protected:
    typedef C Constructor;
    typedef std::map<std::string, Constructor> Constructors;

    static PluginManagerImp<M,C>& instance(const std::string& full_name) {
        static PluginManagerImp<M,C> i(full_name);
        return i;
    }

protected:
    PluginManagerImp(const std::string& full_name)
        : loader_(new Loader("csapex", full_name)), plugins_loaded_(false)
    {
    }

    PluginManagerImp(const PluginManagerImp& rhs);
    PluginManagerImp& operator = (const PluginManagerImp& rhs);

protected:
    virtual ~PluginManagerImp() {}

    void registerConstructor(Constructor constructor) {
        available_classes[constructor.getType()] = constructor;
    }

    void reload() {
        try {
            if(plugins_loaded_) {
                /// @TODO: init library
            }

            std::vector<std::string> classes = loader_->getDeclaredClasses();
            for(std::vector<std::string>::iterator c = classes.begin(); c != classes.end(); ++c) {
//                std::cout << "loading " << typeid(M).name() << " class " << *c << std::endl;

                loader_->loadLibraryForClass(*c);

                plugin_manager::InstallConstructor<M>::installConstructor(this, loader_, *c);

                Constructor constructor;
                constructor.setType(*c);
                constructor.setConstructor(boost::bind(&Loader::createUnmanagedInstance, loader_, *c));
                registerConstructor(constructor);
            }
        } catch(pluginlib::PluginlibException& ex) {
            std::cerr << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
        }
        plugins_loaded_ = true;
    }

protected:
    Loader* loader_;
    bool plugins_loaded_;

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

    virtual ~PluginManager()
    {}

    virtual void registerConstructor(Constructor constructor) {
        instance.registerConstructor(constructor);
    }

    virtual bool pluginsLoaded() const {
        return instance.plugins_loaded_;
    }

    virtual void reload() {
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

#endif // PLUGIN_MANAGER_HPP
