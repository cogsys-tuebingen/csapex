/// HEADER
#include <csapex/manager/message_provider_manager.h>

/// COMPONENT
#include <csapex/plugin/plugin_manager.hpp>
#include <csapex/core/settings.h>
#include <csapex/msg/apex_message_provider.h>

/// SYSTEM
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;
using namespace csapex;

MessageProviderManager::MessageProviderManager() : manager_(new PluginManager<MessageProvider>("csapex::MessageProvider"))
{
}

MessageProviderManager::~MessageProviderManager()
{
}

void MessageProviderManager::setPluginLocator(PluginLocatorPtr locator)
{
    plugin_locator_ = locator;
}

void MessageProviderManager::shutdown()
{
    classes.clear();
    plugin_locator_.reset();
    manager_.reset();
}

void MessageProviderManager::loadPlugins()
{
    manager_->load(plugin_locator_.get());

    classes.clear();

    supported_types_ = std::string("*") + Settings::message_extension + " ";
    registerMessageProvider(Settings::message_extension, std::bind(&ApexMessageProvider::make));
    registerMessageProvider(Settings::message_extension_compressed, std::bind(&ApexMessageProvider::make));
    registerMessageProvider(Settings::message_extension_binary, std::bind(&ApexMessageProvider::make));

    for (const auto& pair : manager_->getConstructors()) {
        try {
            MessageProvider::Ptr prov(pair.second());
            for (const std::string& extension : prov->getExtensions()) {
                std::string ext = extension;
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                registerMessageProvider(ext, pair.second);

                supported_types_ += std::string("*") + ext + " ";
            }
        } catch (const std::exception& e) {
            std::cerr << "cannot load message provider " << pair.first << ": " << typeid(e).name() << ", what=" << e.what() << std::endl;
        }
    }

    supported_types_ = supported_types_.substr(0, supported_types_.length() - 1);
}

MessageProvider::Ptr MessageProviderManager::createMessageProvider(const std::string& path)
{
    return instance().createMessageProviderHelper(path);
}

MessageProvider::Ptr MessageProviderManager::createMessageProviderHelper(const std::string& path)
{
    if (!manager_->pluginsLoaded()) {
        loadPlugins();
    }

    if (path.empty()) {
        return nullptr;
    }

    if (classes.empty()) {
        throw std::runtime_error("no message providers registered!");
    }

    bfs::path file(path);

    bool is_dir = bfs::is_directory(file);
    std::string ext = is_dir ? ".DIRECTORY" : file.extension().string();

    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    auto pos = classes.find(ext);
    if (pos == classes.end()) {
        // try with one dot in the extension
        std::string filename = file.filename().string();
        std::size_t dot_pos = filename.rfind('.');
        if (dot_pos != std::string::npos) {
            std::size_t ext_pos = filename.rfind('.', dot_pos - 1);
            if (ext_pos != std::string::npos) {
                std::string ext = filename.substr(ext_pos);
                pos = classes.find(ext);
            }
        }
    }

    if (pos == classes.end()) {
        throw std::runtime_error(std::string("cannot import ") + path);
    }

    return (pos->second)();
}

void MessageProviderManager::registerMessageProvider(const std::string& type, Constructor constructor)
{
    instance().classes[type] = constructor;
}

std::string MessageProviderManager::supportedTypes()
{
    if (!manager_->pluginsLoaded()) {
        loadPlugins();
    }

    return supported_types_;
}
