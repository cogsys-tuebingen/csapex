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
    while(!boot_plugin_loaders_.empty()) {
        delete boot_plugin_loaders_.front();
        boot_plugin_loaders_.erase(boot_plugin_loaders_.begin());
    }
}

void Bootstrap::bootFrom(const std::string& dir_name,
                         PluginLocator* plugin_locator)
{
    boost::filesystem::path directory(dir_name);

    if(boost::filesystem::exists(directory)) {
        boost::filesystem::directory_iterator dir(directory);
        boost::filesystem::directory_iterator end;

        for(; dir != end; ++dir) {
            boost::filesystem::path path = dir->path();

            Bootstrap::boot_plugin_loaders_.push_back(new class_loader::ClassLoader(path.string()));
            class_loader::ClassLoader* loader = Bootstrap::boot_plugin_loaders_.back();

            try {
                apex_assert_hard(loader->isLibraryLoaded());
                std::vector<std::string> classes = loader->getAvailableClasses<BootstrapPlugin>();
                for(std::size_t c = 0; c < classes.size(); ++c){
                    auto boost_plugin = loader->createInstance<BootstrapPlugin>(classes[c]);
                    std::shared_ptr<BootstrapPlugin> plugin = shared_ptr_tools::to_std_shared(boost_plugin);
                    Bootstrap::boot_plugins_.push_back(plugin);

                    plugin->boot(plugin_locator);
                }
            } catch(const std::exception& e) {
                std::cerr << "boot plugin " << path << " failed: " << e.what() << std::endl;
                std::abort();
            }
        }
    }

    if (Bootstrap::boot_plugins_.empty()) {
        std::cerr << "SEVERE WARNING: there is no boot plugin in directory " << dir_name << '\n';
        std::cerr << "please create it by either" << '\n';
        std::cerr << "a) running the following command:" << '\n';
        std::cerr << "   for dir in ${LD_LIBRARY_PATH//:/ }; do \\\n"
                     "     path=$(find $dir -name libcsapex_ros_boot.so);\\\n"
                     "     if [ $path ]; then mkdir -p ~/.csapex/boot &&  ln -sf $path ~/.csapex/boot/libcsapex_ros_boot.so; fi;\\\n"
                     "   done" << '\n';
        std::cerr << "b) creating a link by hand in ~/.csapex/boot/ to the library 'libcsapex_ros_boot.so' " << '\n';
        std::cerr << "c) recompiling csapex_ros" << '\n';
        std::cerr << std::flush;
    }
}
