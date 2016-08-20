#ifndef PLUGIN_MANAGER_HPP
#define PLUGIN_MANAGER_HPP

/// COMPONENT
#include <csapex/utility/constructor.hpp>
#include <csapex/plugin/plugin_locator.h>
#include <csapex/plugin/plugin_constructor.hpp>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>
#include <class_loader/class_loader.h>
#include <set>
#if WIN32
#define TIXML_USE_STL
#include <tinyxml/tinyxml.h>
#else
#include <tinyxml.h>
#endif
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
    
    bool processManifest(csapex::PluginLocator* locator, const std::string& xml_file)
    {
        TiXmlDocument document;
        document.LoadFile(xml_file);
        TiXmlElement * config = document.RootElement();
        if (config == nullptr) {
            std::cerr << "[Plugin] Cannot load the file " << xml_file << std::endl;
            return false;
        }

        TiXmlElement* library = config;
        if (library->ValueStr() != "library") {
            library = library->NextSiblingElement("library");
        }        
        while (library != nullptr) {
            
            std::string library_name = library->Attribute("path");
            if (library_name.size() == 0) {
                std::cerr << "[Plugin] Item in row" << library->Row() << " does not contain a path attribute" << std::endl;
                continue;
            }
            
            if(!locator->isLibraryIgnored(library_name)) {
                loadLibrary(library_name, library);
            }

            library_to_locator_[library_name] = locator;
            
            library = library->NextSiblingElement( "library" );
        }

        manifest_loaded(xml_file, config);
        
        return true;
    }

    void loadLibrary(const std::string& library_name, TiXmlElement* library)  {
        if(!containsPlugins(library)) {
        }

        TiXmlElement* class_element = library->FirstChildElement("class");
        while (class_element) {
            loadClass(library_name, class_element);

            class_element = class_element->NextSiblingElement( "class" );
        }
    }

    
    void loadClass(const std::string& library_name, TiXmlElement* class_element) {
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
            
            constructor.setConstructor([this, lookup_name]() {
                return createInstance(lookup_name);
            });
            constructor.setLibraryName(library_name);
            
            registerConstructor(constructor);

            plugin_to_library_[lookup_name] = library_name;
        }
    }

    std::shared_ptr<M> createInstance(const std::string& lookup_name)
    {
        auto loader = getLoader(plugin_to_library_.at(lookup_name));
        if(!loader) {
            std::cerr << "cannot create instance of " << lookup_name << ": no loader exists" << std::endl;
            return nullptr;
        } else if(!loader->template isClassAvailable<M>(lookup_name)) {
            std::cerr << "cannot create instance of " << lookup_name << ": class is not available" << std::endl;
            return nullptr;
        }
        try {
            M* raw_ptr = loader->template createUnmanagedInstance<M>(lookup_name);
            return std::shared_ptr<M>(raw_ptr);

        } catch(const std::exception& e) {
            std::cerr << "cannot create instance of " << lookup_name << ": " << e.what() << std::endl;
            return nullptr;
        }
    }

    std::shared_ptr<class_loader::ClassLoader> getLoader(const std::string& library_name)
    {
#if WIN32
		std::string library_path = library_name.substr(3) + ".dll";
#else
        std::string library_path = library_name + ".so";
#endif

        auto pos = loaders_.find(library_path);
        if(pos == loaders_.end()) {
            try {
                auto loader = std::make_shared<class_loader::ClassLoader>(library_path);
                library_to_locator_[library_name]->setLibraryLoaded(library_name, library_path);

                loaders_[library_path] = loader;

                return loader;

            } catch(const class_loader::ClassLoaderException& e) {
                std::cerr << "cannot load library " << library_name << ": " << e.what() << std::endl;
                library_to_locator_[library_name]->setLibraryError(library_name, e.what());
            }

            return nullptr;


        } else {
            return pos->second;
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


    bool containsPlugins(TiXmlElement* library)
    {
        TiXmlElement* class_element = library->FirstChildElement("class");
        while (class_element) {
            std::string base_class_type = class_element->Attribute("base_class_type");
            if(base_class_type == full_name_) {
                return true;
            }
            class_element = class_element->NextSiblingElement( "class" );
        }

        return false;
    }

protected:
    csapex::slim_signal::Signal<void(const std::string&)> loaded;
    csapex::slim_signal::Signal<void(const std::string& file, const TiXmlElement* document)> manifest_loaded;
    
protected:
    bool plugins_loaded_;

    std::map< std::string, std::shared_ptr<class_loader::ClassLoader> > loaders_;
    std::map< std::string, std::string > plugin_to_library_;
    
    std::vector<std::string> library_paths_;
    std::map<std::string, csapex::PluginLocator*> library_to_locator_;
    
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
        instance->manifest_loaded.connect(manifest_loaded);
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
        
        instance->load(locator);
    }
    
    const Constructors& getConstructors() const {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        return instance->available_classes;
    }
    const Constructor& getConstructor(unsigned index) const {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        typename Constructors::iterator it = instance->available_classes.begin();
        std::advance(it, index);
        return it->second;
    }
    const Constructor& getConstructor(const std::string& key) const {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        return instance->available_classes.at(key);
    }
    Constructor& getConstructor(const std::string& key) {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        return instance->available_classes.at(key);
    }

    Constructor* getConstructorNoThrow(const std::string& key) {
        std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
        auto pos = instance->available_classes.find(key);
        if(pos != instance->available_classes.end()) {
            return &pos->second;
        } else {
            return nullptr;
        }
    }

public:
    csapex::slim_signal::Signal<void(const std::string&)> loaded;
    csapex::slim_signal::Signal<void(const std::string& file, const TiXmlElement* document)> manifest_loaded;
    
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
