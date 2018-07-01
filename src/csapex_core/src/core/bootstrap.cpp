/// HEADER
#include <csapex/core/bootstrap.h>

/// PROJECT
#include <csapex/core/bootstrap_plugin.h>
#include <csapex/plugin/plugin_locator.h>
#include <csapex/plugin/plugin_manager.hpp>
#include <csapex/utility/shared_ptr_tools.hpp>

/// SYSTEM
#include <fstream>
#ifdef WIN32
#include <direct.h>
#endif
#include <boost/filesystem.hpp>

using namespace csapex;

Bootstrap::Bootstrap()
{
}

Bootstrap::~Bootstrap()
{
    boot_plugins_.clear();
    while (!boot_plugin_loaders_.empty()) {
        delete boot_plugin_loaders_.front();
        boot_plugin_loaders_.erase(boot_plugin_loaders_.begin());
    }
}

bool Bootstrap::bootFrom(const std::string& default_dir_name, PluginLocator* plugin_locator, bool require_boot_plugin)
{
    const char* env = getenv("CSAPEX_BOOT_DIRECTORY");
    if (env != nullptr) {
        return tryBootFrom(env, plugin_locator, require_boot_plugin);
    } else {
        return tryBootFrom(default_dir_name, plugin_locator, require_boot_plugin);
    }
}

bool Bootstrap::tryBootFrom(const std::string& dir_name, PluginLocator* plugin_locator, bool require_boot_plugin)
{
    boost::filesystem::path directory(dir_name);

    if (boost::filesystem::exists(directory)) {
        boost::filesystem::directory_iterator dir(directory);
        boost::filesystem::directory_iterator end;

        for (; dir != end; ++dir) {
            boost::filesystem::path path = dir->path();

            class_loader::ClassLoader* loader = nullptr;

            std::string extension = boost::filesystem::extension(path);
            std::transform(extension.begin(), extension.end(), extension.begin(), tolower);

            // only load boot plugins from shared libraries
#ifdef WIN32
            if (extension != ".dll") {
#else
            if (extension != ".so") {
#endif
                continue;
            }

            // only load boot plugins if boot is part of the name
            std::string filename = path.filename().string();
            if (filename.find("boot") >= filename.npos) {
                continue;
            }

            try {
                loader = new class_loader::ClassLoader(path.string());

            } catch (const class_loader::LibraryLoadException& e) {
                continue;
            }

            apex_assert_hard(loader);
            bool plugin_loaded = false;

            try {
                std::vector<std::string> classes = loader->getAvailableClasses<BootstrapPlugin>();

                for (std::size_t c = 0; c < classes.size(); ++c) {
                    apex_assert_hard(loader->isClassAvailable<BootstrapPlugin>(classes[c]));
                    auto boost_plugin = loader->createInstance<BootstrapPlugin>(classes[c]);

                    std::shared_ptr<BootstrapPlugin> plugin = shared_ptr_tools::to_std_shared(boost_plugin);
                    Bootstrap::boot_plugins_.push_back(plugin);

                    plugin_loaded = true;

                    plugin->boot(plugin_locator);
                }
            } catch (const std::exception& e) {
                std::cerr << "boot plugin " << path << " failed: " << e.what() << std::endl;
                std::abort();
            }

            if (plugin_loaded) {
                Bootstrap::boot_plugin_loaders_.push_back(loader);
            } else {
                std::cerr << "Library " << path << " does not contain any boot plugins" << std::endl;
                delete loader;
            }
        }
    } else if (require_boot_plugin) {
        std::cerr << "SEVERE ERROR: Boot directory " << dir_name << " does not exist" << std::endl;
        std::cerr << "Set the environment variable CSAPEX_BOOT_DIRECTORY to a directory containing a boot plugin" << '\n';
        return false;
    }

    if (require_boot_plugin && Bootstrap::boot_plugins_.empty()) {
        std::cerr << "SEVERE ERROR: No boot plugins have been detected in directory " << dir_name << '\n';
        std::cerr << "Please make at least one build plugin available, by either:" << '\n';
        std::cerr << "a) running the following command:" << '\n';
        std::cerr << "   for dir in ${LD_LIBRARY_PATH//:/ }; do \\\n"
                     "     path=$(find $dir -name libcsapex_ros_boot.so);\\\n"
                     "     if [ $path ]; then mkdir -p ~/.csapex/boot &&  ln -sf $path ~/.csapex/boot/libcsapex_ros_boot.so; fi;\\\n"
                     "   done"
                  << '\n';
        std::cerr << "b) creating a link by hand in ~/.csapex/boot/ to the library 'libcsapex_ros_boot.so' " << '\n';
        std::cerr << "c) recompiling csapex_ros" << '\n';
        std::cerr << "c) setting the environment variable CSAPEX_BOOT_DIRECTORY to a directory containing a boot plugin" << '\n';
        std::cerr << std::flush;
        return false;
    }

    return true;
}
