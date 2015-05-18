#ifndef PLUGIN_MANAGER_HPP
#define PLUGIN_MANAGER_HPP

/// COMPONENT
#include <csapex/utility/constructor.hpp>
#include <csapex/plugin/plugin_locator.h>
#include <csapex/plugin/plugin_constructor.hpp>

/// SYSTEM
#include <boost/signals2.hpp>
#include <class_loader/class_loader.h>
#include <set>
#include <tinyxml.h>
#include <mutex>

namespace csapex
{

template <class M>
class PluginManagerImp
{
    template <class>
    friend class PluginManager;

protected:
    typedef PluginConstructor<M> PluginConstructorM;
    typedef std::map<std::string, PluginConstructorM> Constructors;

protected:
    PluginManagerImp(const std::string& full_name)
        : plugins_loaded_(false), full_name_(full_name)
    {
    }

    PluginManagerImp(const PluginManagerImp& rhs);
    PluginManagerImp& operator = (const PluginManagerImp& rhs);

public:
    virtual ~PluginManagerImp() {
    }

protected:
    void registerConstructor(PluginConstructorM constructor) {
        available_classes[constructor.getType()] = constructor;
    }

    void enumerateXmlFiles()
    {
    }

    void load(csapex::PluginLocator* locator) {
        std::vector<std::string> xml_files = locator->enumerateXmlFiles<M>();
        std::vector<std::string> library_paths = locator->enumerateLibraryPaths();

        library_paths_.insert(library_paths_.end(), library_paths.begin(), library_paths.end());

        for(std::vector<std::string>::const_iterator manifest = xml_files.begin(); manifest != xml_files.end(); ++manifest) {
            processManifest(locator, *manifest);
        }

        plugins_loaded_ = true;
    }

    void unload(const std::string& library) {
        // unload all matching classes
        for(typename Constructors::iterator it = available_classes.begin(); it != available_classes.end(); ++it) {
            PluginConstructorM& c = it->second;
            if(c.getLibraryName() == library) {
                c.unload();
            }
        }

        std::shared_ptr<class_loader::ClassLoader> loader = loaders_[library];
        assert(!loader->isOnDemandLoadUnloadEnabled());

//        std::cerr << "unloading " << library << " for " << full_name_ << std::endl;
        int retries = 1;
        while(retries != 0) {
            retries = loader->unloadLibrary();
            if(retries != 0) {
//                std::cerr << "there are still " << retries << " unloads necessary to unload " << full_name_ << std::endl;
            }
        }
        if(loader->isLibraryLoadedByAnyClassloader()) {
//            std::cerr << "there still instances of " << library << std::endl;
        } else {
//            std::cerr << "there no more instances of " << library << std::endl;
        }
    }

    void reload(const std::string& library) {
        std::shared_ptr<class_loader::ClassLoader> loader = loaders_[library];
//        std::cerr << "loading " << library  << " for " << full_name_ << std::endl;
        loader->loadLibrary();

        // reload all matching classes
        for(typename Constructors::iterator it = available_classes.begin(); it != available_classes.end(); ++it) {
            PluginConstructorM& c = it->second;
            if(c.getLibraryName() == library) {
                c.reload();
            }
        }

    }

    bool processManifest(csapex::PluginLocator* locator, const std::string& xml_file)
    {
        TiXmlDocument document;
        document.LoadFile(xml_file);
        TiXmlElement * config = document.RootElement();
        if (config == nullptr) {
            std::cerr << "[Plugin] Cannot load the file " << xml_file << std::endl;
            return false;
        }
        if (config->ValueStr() != "library") {
            std::cerr << "[Plugin] Manifest root is not <library>" << std::endl;
            return false;
        }

        TiXmlElement* library = config;
        while (library != nullptr) {

            std::string library_name = library->Attribute("path");
            if (library_name.size() == 0) {
                std::cerr << "[Plugin] Item in row" << library->Row() << " does not contain a path attribute" << std::endl;
                continue;
            }

            if(!locator->isLibraryIgnored(library_name)) {
                try {
                    std::string file = loadLibrary(library_name, library);
                    locator->setLibraryLoaded(library_name, file);

                } catch(const class_loader::ClassLoaderException& e) {
                    std::cerr << "cannot load library " << library_name << ": " << e.what() << std::endl;
                    locator->setLibraryError(library_name, e.what());
                }
            }

            library = library->NextSiblingElement( "library" );
        }

        return true;
    }

    std::string loadLibrary(const std::string& library_name, TiXmlElement* library)  {
        std::string library_path = library_name + ".so";

        std::shared_ptr<class_loader::ClassLoader> loader(new class_loader::ClassLoader(library_path));
        loaders_[library_name] = loader;

        TiXmlElement* class_element = library->FirstChildElement("class");
        while (class_element) {
            loadClass(library_name, class_element, loader.get());

            class_element = class_element->NextSiblingElement( "class" );
        }

        return library_path;
    }

    void loadClass(const std::string& library_name, TiXmlElement* class_element, class_loader::ClassLoader* loader) {
        std::string base_class_type = class_element->Attribute("base_class_type");
        std::string derived_class = class_element->Attribute("type");

        std::string lookup_name;
        if(class_element->Attribute("name") != nullptr) {
            lookup_name = class_element->Attribute("name");
        } else {
            lookup_name = derived_class;
        }

        if(base_class_type == full_name_){
            std::string description = readString(class_element, "description");
            std::string icon = readString(class_element, "icon");
            std::string tags = readString(class_element, "tags");

            PluginConstructorM constructor;
            constructor.setType(lookup_name);
            constructor.setDescription(description);
            constructor.setIcon(icon);
            constructor.setTags(tags);

            std::function< std::shared_ptr<M>(M*)> make_shared_ptr = [](M* p) { return std::shared_ptr<M>(p); };

            auto ptr_maker = std::bind(&class_loader::ClassLoader::createUnmanagedInstance<M>, loader, lookup_name);
            auto shared_ptr_maker = std::bind(make_shared_ptr, ptr_maker);
            constructor.setConstructor(shared_ptr_maker);
            constructor.setLibraryName(library_name);

            registerConstructor(constructor);
        }
    }

    std::string readString(TiXmlElement* class_element, const std::string& name) {
        TiXmlElement* description = class_element->FirstChildElement(name);
        std::string description_str;
        if(description) {
            description_str = description->GetText() ? description->GetText() : "";
        }

        return description_str;
    }

protected:
    boost::signals2::signal<void(const std::string&)> loaded;

protected:
    bool plugins_loaded_;

    std::map< std::string, std::shared_ptr<class_loader::ClassLoader> > loaders_;

    std::vector<std::string> library_paths_;

    std::string full_name_;
    Constructors available_classes;
};

template <typename T>
struct PluginManagerGroup
{
    enum Group {
        value = 10
    };
};

class PluginManagerLocker
{
public:
    static std::mutex& getMutex()
    {
        static PluginManagerLocker instance;
        return instance.mutex;
    }

private:
    PluginManagerLocker()
    {}

private:
    std::mutex mutex;
};

template <class M>
class PluginManager
{
protected:
    typedef PluginManagerImp<M> Parent;

public:
    typedef typename Parent::PluginConstructorM Constructor;
    typedef typename Parent::Constructors Constructors;

    PluginManager(const std::string& full_name)
    {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        if(i_count == 0) {
            ++i_count;
            instance = new Parent(full_name);
        }
        instance->loaded.connect(loaded);
    }

    virtual ~PluginManager()
    {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        --i_count;
        if(i_count == 0) {
            delete instance;
        }
    }

    virtual bool pluginsLoaded() const {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        return instance->plugins_loaded_;
    }

    virtual void load(csapex::PluginLocator* locator) {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());

        locator->unload_library_request.connect(PluginManagerGroup<M>::value, std::bind(&PluginManager<M>::unload, this, std::placeholders::_1), boost::signals2::at_front);
        locator->reload_library_request.connect(-PluginManagerGroup<M>::value, std::bind(&PluginManager<M>::reload, this, std::placeholders::_1), boost::signals2::at_back);

        instance->load(locator);
    }

    void unload(const std::string& library) {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        instance->unload(library);
    }

    void reload(const std::string& library) {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        instance->reload(library);
    }

    const Constructors& availableClasses() const {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        return instance->available_classes;
    }
    const Constructor& availableClasses(unsigned index) const {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        typename Constructors::iterator it = instance->available_classes.begin();
        std::advance(it, index);
        return it->second;
    }
    const Constructor& availableClasses(const std::string& key) const {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        return instance->available_classes[key];
    }
    Constructor& availableClasses(const std::string& key) {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
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
typename PluginManager<M>::Parent* PluginManager<M>::instance(nullptr);

}

#endif // PLUGIN_MANAGER_HPP
