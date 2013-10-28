/// HEADER
#include <csapex/manager/message_provider_manager.h>

/// COMPONENT
#include <utils_plugin/plugin_manager.hpp>

/// SYSTEM
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;
using namespace csapex;

MessageProviderManager::MessageProviderManager()
    : manager_(new PluginManager<MessageProvider>("csapex::MessageProvider"))
{
}

MessageProviderManager::~MessageProviderManager()
{
    delete manager_;
}

void MessageProviderManager::fullReload()
{
    instance().manager_->reload();

    classes.clear();

    supported_types_ = "";

    typedef std::pair<std::string, PluginManager<csapex::MessageProvider>::Constructor> PAIR;
    foreach(PAIR pair, manager_->availableClasses()) {
        try {
            MessageProvider::Ptr prov(pair.second());
            foreach(const std::string& extension, prov->getExtensions()) {
                std::string ext = extension;
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                registerMessageProvider(ext, pair.second);

                supported_types_ += std::string("*") + ext + " ";
            }
        } catch(const std::exception& e) {
            std::cerr << "cannot load message provider " << pair.first << ": " << typeid(e).name() << ", what=" << e.what() << std::endl;
        }
    }

    supported_types_ = supported_types_.substr(0, supported_types_.length()-1);
}

MessageProvider::Ptr MessageProviderManager::createMessageProvider(const std::string& path)
{
    return instance().createMessageProviderHelper(path);
}

MessageProvider::Ptr MessageProviderManager::createMessageProviderHelper(const std::string& path)
{
    if(!manager_->pluginsLoaded()) {
        fullReload();
    }

    bfs::path file(path);

    bool is_dir = bfs::is_directory(file) ;
    std::string ext = is_dir ? ".DIRECTORY" : file.extension().string();

    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if(path.empty()) {
        return MessageProviderNullPtr;
    }

    std::cout << "extension is " << ext << std::endl;

    if(classes.empty()) {
        throw std::runtime_error("no message providers registered!");
    }

    return classes[ext]();
}

void MessageProviderManager::registerMessageProvider(const std::string &type, Constructor constructor)
{
    instance().classes[type] = constructor;
}

std::string MessageProviderManager::supportedTypes()
{
    if(!manager_->pluginsLoaded()) {
        fullReload();
    }

    return supported_types_;
}
