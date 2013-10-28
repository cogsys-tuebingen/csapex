#ifndef PLUGIN_MANAGER_HPP
#define PLUGIN_MANAGER_HPP

/// COMPONENT
#include <utils_plugin/constructor.hpp>
#include <utils_plugin/plugin_loader.h>

/// SYSTEM
#include <boost/signals2.hpp>
#include <pluginlib/class_loader.h>
#include <set>

template <class M>
class PluginManagerImp
{
    template <class>
    friend class PluginManager;

public:
    typedef pluginlib::ClassLoader<M> Loader;

protected:
    typedef DefaultConstructor<M> Constructor;
    typedef std::map<std::string, Constructor> Constructors;

protected:
    PluginManagerImp(const std::string& full_name)
        : loader_("csapex", full_name), plugins_loaded_(false), full_name_(full_name)
    {
    }

    PluginManagerImp(const PluginManagerImp& rhs);
    PluginManagerImp& operator = (const PluginManagerImp& rhs);

public:
    virtual ~PluginManagerImp() {
    }

protected:
    void registerConstructor(Constructor constructor) {
        available_classes[constructor.getType()] = constructor;
    }
    void reload() {
        std::vector<std::string> classes = loader_.getDeclaredClasses();
        for(std::vector<std::string>::iterator c = classes.begin(); c != classes.end(); ++c) {
            // std::cout << "loading " << typeid(M).name() << " class " << *c << std::endl;
            std::string msg = std::string("loading ") + *c;
            loaded(msg);

            try {
                if(!loader_.isClassLoaded(*c)) {
                    loader_.loadLibraryForClass(*c);
                }

                Constructor constructor;
                constructor.setType(*c);
                constructor.setDescription(loader_.getClassDescription(*c));
                constructor.setConstructor(boost::bind(&Loader::createInstance, &loader_, *c));

                registerConstructor(constructor);
            } catch(const pluginlib::PluginlibException& ex) {
                std::cerr << "The plugin " << *c << " failed to load for some reason. Error: " << ex.what() << std::endl;
            } catch(const std::exception& ex) {
                std::cerr << "The plugin " << *c << " failed to load for some reason. Error: " << ex.what() << std::endl;
            }
        }
    }

protected:
    boost::signals2::signal<void(const std::string&)> loaded;

protected:
    Loader loader_;
    bool plugins_loaded_;

    std::string full_name_;
    Constructors available_classes;
};

template <class M>
class PluginManager
{
protected:
    typedef PluginManagerImp<M> Parent;

public:
    typedef typename Parent::Constructor Constructor;
    typedef typename Parent::Constructors Constructors;

    PluginManager(const std::string& full_name)
    {
        if(i_count == 0) {
            ++i_count;
            instance = new Parent(full_name);
        }
        instance->loaded.connect(loaded);
    }

    virtual ~PluginManager()
    {
        --i_count;
        if(i_count == 0) {
            delete instance;
        }
    }

    virtual bool pluginsLoaded() const {
        return instance->plugins_loaded_;
    }

    virtual void reload() {
        instance->reload();
    }

    const Constructors& availableClasses() const {
        return instance->available_classes;
    }
    const Constructor& availableClasses(unsigned index) const {
        typename Constructors::iterator it = instance->available_classes.begin();
        std::advance(it, index);
        return it->second;
    }
    const Constructor& availableClasses(const std::string& key) const {
        return instance->available_classes[key];
    }
    Constructor& availableClasses(const std::string& key) {
        return instance->available_classes[key];
    }

public:
    boost::signals2::signal<void(const std::string&)> loaded;

protected:
    static int i_count;
    static Parent* instance;
};

template <class M>
int PluginManager<M>::i_count = 0;
template <class M>
typename PluginManager<M>::Parent* PluginManager<M>::instance(NULL);

#endif // PLUGIN_MANAGER_HPP
