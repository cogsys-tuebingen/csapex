#ifndef PLUGIN_LOCATOR_H
#define PLUGIN_LOCATOR_H

/// SYSTEM
#include <string>
#include <map>
#include <typeinfo>
#include <vector>
#include <boost/function.hpp>
#include <iostream>

namespace csapex
{
class PluginLocator
{
public:
    template <typename PluginType>
    std::vector<std::string> enumerateXmlFiles() {
        std::map<const std::type_info*, std::vector<boost::function<void(std::vector<std::string>&)> > >::iterator pos = locators_.find(&typeid(PluginType));
        std::vector<std::string> files;
        if(pos != locators_.end()) {
            std::vector<boost::function<void(std::vector<std::string>&)> >& vec = pos->second;
            for(std::size_t i = 0, total = vec.size(); i < total; ++i) {
                vec.at(i)(files);
            }
        }
        return files;
    }

    template <typename PluginType>
    void registerLocator(boost::function<void(std::vector<std::string>&)> fn)
    {
        locators_[&typeid(PluginType)].push_back(fn);
    }

private:
    std::map<const std::type_info*, std::vector<boost::function<void(std::vector<std::string>&)> > > locators_;
};
}

#endif // PLUGIN_LOCATOR_H
