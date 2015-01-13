/// HEADER
#include <csapex/plugin/plugin_locator.h>

/// PROJECT
#include <csapex/model/node.h>
#include <utils_param/string_list_parameter.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/q_signal_relay.h>

/// SYSTEM
#include <istream>
#include <boost/algorithm/string.hpp>
#include <QFileSystemWatcher>
#include <QFile>
#include <thread>

using namespace csapex;

PluginLocator::PluginLocator(Settings &settings)
    : settings_(settings)
{
    if(settings_.knows("ignored_libraries")) {
        ignored_persistent_ = std::dynamic_pointer_cast<param::StringListParameter>(settings_.get("ignored_libraries"));

    } else {
        std::vector<std::string> ignored_libraries(ignored_libraries_.begin(), ignored_libraries_.end());

        ignored_persistent_.reset(new param::StringListParameter("ignored_libraries", param::ParameterDescription("ignored libraries")));
        settings_.add(ignored_persistent_);
    }
    std::vector<std::string> tmp = ignored_persistent_->getValues();
    ignored_libraries_.insert(tmp.begin(), tmp.end());

    std::string ld_lib = getenv("LD_LIBRARY_PATH");
    boost::algorithm::split(library_paths_, ld_lib, boost::is_any_of(":"));
}

PluginLocator::~PluginLocator()
{
    clearFileWatcherCallbacks();
}

std::vector<std::string> PluginLocator::enumerateLibraryPaths()
{
    return library_paths_;
}

void PluginLocator::reloadLibraryIfExists(const std::string &name, const std::string &abs_path)
{
    QFile qfile(QString::fromStdString(abs_path));
    if(qfile.exists()) {
        std::chrono::milliseconds dura(100);

        std::this_thread::sleep_for(dura);
        while(qfile.size() == 0) {
            std::this_thread::sleep_for(dura);
        }
        reloadLibrary(name);
    }
}

void PluginLocator::reloadLibrary(const std::string &name)
{
    std::cout << "WARNING: reloading plugin " << name << std::endl;
    unload_library_request(name);

    reload_library_request(name);

    reload_done();

    if(isAutoReload()) {
        createFileWatcher(name);
    }
}

void PluginLocator::setAutoReload(bool autoreload)
{
    if(isAutoReload() != autoreload) {
        settings_.set("auto_reload_libraries", autoreload);

        library_watchers_.clear();
        clearFileWatcherCallbacks();

        if(autoreload) {
            foreach(const std::string& name, loaded_libraries_) {
                createFileWatcher(name);
            }
        }
    }
}

bool PluginLocator::isAutoReload() const
{
    return settings_.get("auto_reload_libraries", false);
}

void PluginLocator::ignoreLibrary(const std::string &name, bool ignore)
{
    if(ignore) {
        ignored_libraries_.insert(name);
        ignored_persistent_->add(name);
    } else {
        ignored_libraries_.erase(name);
        ignored_persistent_->removeAll(name);
    }
}

bool PluginLocator::isLibraryIgnored(const std::string &name) const
{
    return ignored_libraries_.find(name) != ignored_libraries_.end();
}

void PluginLocator::createFileWatcher(const std::string &name)
{
    const std::string& file = library_file_[name];

    for(int i = -1, total = library_paths_.size(); i < total; ++i) {
        const std::string& path = (i == -1) ? "" : library_paths_[i];
        std::string abs_path = path + "/" + file;

        QFile qfile(QString::fromStdString(abs_path));
        if(qfile.exists()) {
            library_watchers_[name] = std::shared_ptr<QFileSystemWatcher>(new QFileSystemWatcher);
            library_watchers_[name]->addPath(QString::fromStdString(abs_path));

            qt_helper::Call* call = new qt_helper::Call(boost::bind(&PluginLocator::reloadLibraryIfExists, this, name, abs_path));

            QObject::connect(library_watchers_[name].get(), SIGNAL(fileChanged(const QString&)),
                             call, SLOT(call()));
            break;
        }
    }
}

void PluginLocator::clearFileWatcherCallbacks()
{
    while(!callbacks_.empty()) {
        callbacks_.back()->disconnect();
        delete callbacks_.back();
        callbacks_.pop_back();
    }
}

void PluginLocator::setLibraryLoaded(const std::string &name, const std::string& file)
{
    loaded_libraries_.insert(name);
    library_file_[name] = file;

    if(isAutoReload()) {
        createFileWatcher(name);
    }
}

bool PluginLocator::isLibraryLoaded(const std::string &name) const
{
    return loaded_libraries_.find(name) != loaded_libraries_.end();
}


void PluginLocator::setLibraryError(const std::string& name, const std::string& error)
{
    error_libraries_[name] = error;
}

bool PluginLocator::hasLibraryError(const std::string &name) const
{
    return error_libraries_.find(name) != error_libraries_.end();
}

std::string PluginLocator::getLibraryError(const std::string &name) const
{
    return error_libraries_.at(name);
}

std::vector<std::string> PluginLocator::getAllLibraries() const
{
    std::vector<std::string> result(loaded_libraries_.begin(), loaded_libraries_.end());
    for(std::map<std::string, std::string>::const_iterator it = error_libraries_.begin();
        it != error_libraries_.end(); ++it) {
        result.push_back(it->first);
    }

    result.insert(result.end(), ignored_libraries_.begin(), ignored_libraries_.end());

    std::sort(result.begin(), result.end());

    return result;
}
