#ifndef PLUGIN_MANAGER_HPP
#define PLUGIN_MANAGER_HPP

/// COMPONENT
#include <csapex/utility/constructor.hpp>

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
    typedef pluginlib::ClassLoader<M> PluginLibLoader;

protected:
    typedef DefaultConstructor<M> Constructor;
    typedef std::map<std::string, Constructor> Constructors;

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
    void registerConstructor(Constructor constructor) {
        available_classes[constructor.getType()] = constructor;
    }
    void load() {
        // TODO: make this via a plugin! -> csapex_ros
        PluginLibLoader pluginlib_loader("csapex", full_name_);
        xml_files_ = pluginlib_loader.getPluginXmlPaths();

        std::unique(xml_files_.begin(), xml_files_.end());

        for(std::vector<std::string>::const_iterator manifest = xml_files_.begin(); manifest != xml_files_.end(); ++manifest) {
            processManifest(*manifest);
        }

        plugins_loaded_ = true;
    }

    bool processManifest(const std::string& xml_file)
    {
        TiXmlDocument document;
        document.LoadFile(xml_file);
        TiXmlElement * config = document.RootElement();
        if (config == NULL) {
            std::cerr << "[Plugin] Cannot load the file " << xml_file << std::endl;
            return false;
        }
        if (config->ValueStr() != "library") {
            std::cerr << "[Plugin] Manifest root is not <library>" << std::endl;
            return false;
        }

        TiXmlElement* library = config;
        while (library != NULL) {

            std::string library_name = library->Attribute("path");
            if (library_name.size() == 0) {
                std::cerr << "[Plugin] Item in row" << library->Row() << " does not contain a path attribute" << std::endl;
                continue;
            }

            try {
                loadLibrary(library_name, library);
            } catch(const class_loader::ClassLoaderException& e) {
                std::cerr << "cannot load library " << library_name << ": " << e.what() << std::endl;
            }

            library = library->NextSiblingElement( "library" );
        }

        return true;
    }

    void loadLibrary(const std::string& library_name, TiXmlElement* library)  {
        std::string library_path = library_name + ".so";

        boost::shared_ptr<class_loader::ClassLoader> loader(new class_loader::ClassLoader(library_path));
        loaders_[library_path] = loader;


        TiXmlElement* class_element = library->FirstChildElement("class");
        while (class_element) {
            loadClass(class_element, loader.get());

            class_element = class_element->NextSiblingElement( "class" );
        }
    }

    void loadClass(TiXmlElement* class_element, class_loader::ClassLoader* loader) {
        std::string base_class_type = class_element->Attribute("base_class_type");
        std::string derived_class = class_element->Attribute("type");

        std::string lookup_name;
        if(class_element->Attribute("name") != NULL) {
            lookup_name = class_element->Attribute("name");
        } else {
            lookup_name = derived_class;
        }

        if(base_class_type == full_name_){
            std::string description = readString(class_element, "description");
            std::string icon = readString(class_element, "icon");
            std::string tags = readString(class_element, "tags");

            Constructor constructor;
            constructor.setType(lookup_name);
            constructor.setDescription(description);
            constructor.setIcon(icon);
            constructor.setTags(tags);
            constructor.setConstructor(boost::bind(&class_loader::ClassLoader::createInstance<M>, loader, lookup_name));

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

    std::vector<std::string> xml_files_;
    std::map< std::string, boost::shared_ptr<class_loader::ClassLoader> > loaders_;

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

    virtual void load() {
        instance->load();
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
