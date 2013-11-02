#ifndef CONFIG_H
#define CONFIG_H

/// COMPONENT
#include "types.h"
#include <utils_param/parameter_provider.h>
#include <utils_param/parameter_map.h>

/// PROJECT
#include <common/global.hpp>

/// SYSTEM
#include <boost/serialization/split_member.hpp>
#include <boost/signals2.hpp>

namespace impl{
struct ConfigHolder;
}

/**
 * @brief The Config class extends the automatically generated ROS config
 *        and contains settings for every part of the program
 */
class Config : public param::ParameterProvider
{
    friend class boost::serialization::access;
    friend class Reconfigurable;
    friend class impl::ConfigHolder;

protected:
    /**
     * @brief Config
     */
    Config();

public:
    /**
     * @brief applyGlobal replaces all instances of Config with the one the method is called on
     */
    void replaceInstance() const;

    /**
     * @brief get Accessor
     */
    static Config instance();

    /**
     * @brief getDescription returns a descriptive name of this config
     * @return a descriptive name
     */
    std::string getDescription() const;

    /**
     * @brief computeHash returns a hash code for this config
     * @return a hash code
     */
    std::string computeHash() const;

    /**
     * @brief getParameter
     * @param name
     * @return Parameter
     */
    param::Parameter& getParameter(const std::string &name);
    const param::Parameter& getConstParameter(const std::string &name) const;

protected:
    static impl::ConfigHolder& latest_config();


    static boost::signals2::signal<void(const Config&)> replace;

public:
    void setKeypointType(const std::string& type);
    void setDescriptorType(const std::string& type);

protected:
    virtual void make_defaults();
    void init();

private:
    param::ParameterMap parameters;

protected:
    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*file_version*/) {
        //boost::serialization::split_member(ar, *this, file_version);

        ar & parameters;

        setKeypointType(parameters["keypointType"]);
        setDescriptorType(parameters["descriptorType"]);
    }
};

namespace impl{
struct ConfigHolder {
    Config value;
};
}

#endif // CONFIG_H
