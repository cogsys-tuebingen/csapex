#ifndef PLUGIN_H
#define PLUGIN_H

/// COMPONENT
#include "plugin_manager.h"

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/function.hpp>
#include <QLayout>
#include <vector>

#define REGISTER_PLUGIN(class_name, type) \
    namespace vision_evaluator { \
    class _____##type##_##class_name##_registrator { \
        static _____##type##_##class_name##_registrator instance; \
        _____##type##_##class_name##_registrator () {\
            vision_evaluator::Plugin<class_name>::instance().register_constructor(boost::bind(type::createInstance, _1), #class_name, #type);\
            std::cout << "register instance " << #class_name << " with type " << #type << std::endl; \
        } \
    };\
    _____##type##_##class_name##_registrator _____##type##_##class_name##_registrator::instance;\
    }

//  register_constructor(boost::bind(type::createInstance), #class_name, #type);

#define REGISTER_META_PLUGIN(class_name) \
    namespace vision_evaluator { \
    class _____meta_##class_name##_registrator {\
        static _____meta_##class_name##_registrator instance; \
        _____meta_##class_name##_registrator () {\
            vision_evaluator::PluginManager::instance().register_meta_plugin(boost::bind(class_name::init, _1, _2), boost::bind(class_name::createMetaInstance));\
            std::cout << "register meta " << #class_name << std::endl; \
        } \
    };\
    _____meta_##class_name##_registrator _____meta_##class_name##_registrator::instance;\
    }

namespace vision_evaluator
{

template <typename Type>
class Plugin : public PluginBase
{
public:
    enum CONSTRUCTOR_MODE {
        CONSTRUCTOR_INSTANTIATE,
        CONSTRUCTOR_META
    };

    typedef boost::shared_ptr<Type> TypePtr;
    typedef boost::function<TypePtr(CONSTRUCTOR_MODE instantiation)> Constructor;

    static PluginBase::Selector makeSelector() {
        return boost::bind(&Plugin<Type>::compareType, _1);
    }

    static Plugin<Type>& instance() {
        static Plugin<Type> instance("meta");
        return instance;
    }

    void register_constructor(Constructor constructor, const std::string& class_name, const std::string& type) {
        std::cout << "register " << type << " as " << class_name << " > before " << constructors.size() << std::endl;
        constructors.push_back(constructor);
        std::cout << "register " << type << " as " << class_name << " > after  " << constructors.size() << std::endl;
    }

private:
    static bool compareType(PluginBase::PluginPtr other) {
        PluginBase* ptr = other.get();
        return dynamic_cast<Type*>(ptr);
    }

protected:
    Plugin(const std::string& label)
        : PluginBase(label) {
        std::cout << "init " << label << std::endl;
    }

    template <class Container, class PluginType>
    PluginType* get() {
        assert(queue_);
        return queue_->getInstance<Container, PluginType>();
    }

    virtual void insert(QBoxLayout*) {
        // not implemented in base class
    }

public:
    std::vector<Constructor> constructors;
};

} /// NAMESPACE

#endif // PLUGIN_H
