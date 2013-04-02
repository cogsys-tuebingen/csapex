/*
 * generic_manager.hpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef GENERIC_MANAGER_H
#define GENERIC_MANAGER_H

/// SYSTEM
#include <boost/signals2.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/construct.hpp>
#include <pluginlib/class_loader.h>

#define REGISTER_GENERIC(Manager, class_name)\
    namespace vision_evaluator { \
    class _____Manager_##class_name##_registrator { \
        static _____Manager_##class_name##_registrator instance; \
        _____Manager_##class_name##_registrator () {\
            std::cout << "register filter instance " << #class_name << std::endl; \
                Manager::Constructor constructor; \
                constructor.name = #class_name; \
                constructor.constructor = boost::lambda::new_ptr<class_name>(); \
                vision_evaluator::Manager::available_classes.push_back(constructor); \
        } \
    };\
    _____Manager_##class_name##_registrator _____Manager_##class_name##_registrator::instance;\
    }

template <class M>
class GenericManager : public M
{
public:
    struct Constructor {
        std::string name;
        boost::function<M*()> constructor;

        typename M::Ptr operator () (){
            typename M::Ptr res(constructor());
            res->setName(name);
            assert(res.get() != NULL);
            return res;
        }
    };
    typedef pluginlib::ClassLoader<M> Loader;

    static std::vector<Constructor> available_classes;
    static bool plugins_loaded_;
    static boost::shared_ptr<Loader> loader_;

public:
    GenericManager(const std::string& full_name)
    {
        if(loader_ == NULL) {
            loader_.reset(new Loader("vision_evaluator", full_name));
        }
    }

    virtual ~GenericManager() {
    }

    void reload() {
        try
        {
            std::vector<std::string> classes = loader_->getDeclaredClasses();
            for(std::vector<std::string>::iterator c = classes.begin(); c != classes.end(); ++c) {
                std::cout << "load library for class " << *c << std::endl;
                loader_->loadLibraryForClass(*c);

                Constructor constructor;
                constructor.name = *c;
                constructor.constructor = boost::bind(&Loader::createUnmanagedInstance, loader_.get(), *c);
                available_classes.push_back(constructor);
                std::cout << "loaded " << typeid(M).name() << " class " << *c << std::endl;
            }
        }
        catch(pluginlib::PluginlibException& ex)
        {
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
        plugins_loaded_ = true;
    }
};

template <class M>
std::vector<typename GenericManager<M>::Constructor> GenericManager<M>::available_classes;

template <class M>
bool GenericManager<M>::plugins_loaded_ = false;

template <class M>
boost::shared_ptr<typename GenericManager<M>::Loader> GenericManager<M>::loader_((typename GenericManager<M>::Loader*) NULL);

#endif // GENERIC_MANAGER_H
