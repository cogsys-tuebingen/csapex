#ifndef PLUGIN_MANAGER_HPP
#define PLUGIN_MANAGER_HPP

/// COMPONENT
#include <utils_plugin/constructor.hpp>

/// SYSTEM
#include <boost/signals2.hpp>
#include <pluginlib/class_loader.h>

namespace plugin_manager {
template <class C>
struct InstallConstructor
{
    template <class M, class L>
    static void installConstructor(M*, L*, const std::string&, const std::string&) {}
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

//    static PluginManagerImp<M,C>& instance(const std::string& full_name) {
//        static boost::shared_ptr<PluginManagerImp<M,C> > i(new PluginManagerImp<M,C>(full_name));
//        return *i;
//    }

protected:
    PluginManagerImp(const std::string& full_name)
        : loader_("csapex", full_name), plugins_loaded_(false)
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
            //                std::cout << "loading " << typeid(M).name() << " class " << *c << std::endl;
            std::string msg = std::string("loading ") + *c;
            loaded(msg);

            try {
                if(!loader_.isClassLoaded(*c)) {
                    loader_.loadLibraryForClass(*c);
                }

                plugin_manager::InstallConstructor<M>::installConstructor(this, &loader_, *c, loader_.getClassDescription(*c));

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
        plugins_loaded_ = true;
    }

protected:
    boost::signals2::signal<void(const std::string&)> loaded;

protected:
    Loader loader_;
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

    virtual void registerConstructor(Constructor constructor) {
        instance->registerConstructor(constructor);
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
//    Parent& instance;

    static int i_count;
    static Parent* instance;
};

template <class M, class C>
int PluginManager<M,C>::i_count = 0;
template <class M, class C>
typename PluginManager<M,C>::Parent* PluginManager<M,C>::instance(NULL);

#endif // PLUGIN_MANAGER_HPP
