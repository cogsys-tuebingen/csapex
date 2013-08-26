#ifndef EXTRACTOR_MANAGER_H
#define EXTRACTOR_MANAGER_H

/// COMPONENT
#include "extractor.h"

/// PROJECT
#include <utils_plugin/plugin_manager.hpp>
#include <utils_plugin/constructor.hpp>
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <boost/function.hpp>
#include <boost/foreach.hpp>

namespace csapex
{
template <typename Any>
class DetectorTraits
{
    typedef char Small;
    class Big
    {
        char dummy[2];
    };

    template <typename Class> static Small testKeypoint(typeof(&Class::keypoint)) ;
    template <typename Class> static Big testKeypoint(...);

    template <typename Class> static Small testDescriptor(typeof(&Class::descriptor)) ;
    template <typename Class> static Big testDescriptor(...);


public:
    enum { HasKeypoint = sizeof(testKeypoint<Any>(0)) == sizeof(Small) };
    enum { HasDescriptor = sizeof(testDescriptor<Any>(0)) == sizeof(Small) };
};
}

#define EXTRACTOR_IMPLEMENTATION \
    std::string getName() const { return name; }\
    static const std::string name;

#define REGISTER_FEATURE_DETECTOR(class_name, impname)\
    const std::string class_name::name(#impname); \
    namespace csapex { \
    class _____##class_name##impname##_registrator { \
        static _____##class_name##impname##_registrator instance; \
        _____##class_name##impname##_registrator () {\
            registerKeypointConstructor<class_name>();\
            registerDescriptorConstructor<class_name>();\
        }\
        template <class T>\
        static void registerKeypointConstructor(typename boost::enable_if_c<csapex::DetectorTraits<T>::HasKeypoint, T>::type* dummy = 0) {\
            ExtractorManager::instance().registerKeypointConstructor(T::name, boost::bind(&T::keypoint, _1, _2)); \
        }\
        template <class T>\
        static void registerKeypointConstructor(typename boost::disable_if_c<csapex::DetectorTraits<T>::HasKeypoint, T>::type* dummy = 0) {}\
        template <class T>\
        static void registerDescriptorConstructor(typename boost::enable_if_c<csapex::DetectorTraits<T>::HasDescriptor, T>::type* dummy = 0) {\
            ExtractorManager::instance().registerDescriptorConstructor(T::name, boost::bind(&T::descriptor, _1)); \
        }\
        template <class T>\
        static void registerDescriptorConstructor(typename boost::disable_if_c<csapex::DetectorTraits<T>::HasDescriptor, T>::type* dummy = 0) {}\
    };\
    _____##class_name##impname##_registrator _____##class_name##impname##_registrator::instance;\
    }



namespace csapex
{
/**
 * @brief The ExtractorManager class manages all instances of feature detectors and descriptor extractors.
 *        It functions like a proxy to the underlying singleton classes.
 */
class ExtractorManager : public Singleton<ExtractorManager>
{
    friend class Singleton<ExtractorManager>;

public:
    /**
     * @brief The ExtractorInitializer
     */
    struct ExtractorInitializer : public Constructor {
        typedef boost::function<void(Extractor*, bool)> Call;

        void operator()(Extractor* r, bool complete = false) const {
            return construct(r, complete);
        }

        void construct(Extractor* r, bool complete = false) const {
            constructor(r, complete);
        }

        void setConstructor(Call c) {
            constructor = c;
            has_constructor = true;
        }

    private:
        Call constructor;
    };

    class KeypointInitializer : public ExtractorInitializer {};
    class DescriptorInitializer : public ExtractorInitializer {};

    typedef typename KeypointInitializer::Call KeypointInit;
    typedef typename DescriptorInitializer::Call DescriptorInit;

    typedef PluginManager<Extractor, KeypointInitializer> KeypointInitializerManager;
    typedef PluginManager<Extractor, DescriptorInitializer> DescriptorInitializerManager;

private:
    /**
     * @brief ExtractorManager functions like a proxy to the underlying singleton classes.
     */
    ExtractorManager();

public:
    /**
     * @brief ~ExtractorManager
     */
    virtual ~ExtractorManager();

    /**
     * @brief registerKeypointConstructor installs a new keypoint detector instance
     * @param key name of the instance, must be unique
     * @param kc Constructor of the instance, called if <b>key</b> is requested
     */
    void registerKeypointConstructor(const std::string& key, KeypointInit kc);

    /**
     * @brief registerDescriptorConstructor installs a new descriptor extractor instance
     * @param key name of the instance, must be unique
     * @param dc Constructor of the instance, called if <b>key</b> is requested
     */
    void registerDescriptorConstructor(const std::string& key, DescriptorInit dc);

    /**
     * @brief getInitializer get a functor to initialize an Extractor
     * @param keypoint name of the keypoint detector
     * @param descriptor name of the descriptor extractor
     * @throws Extractor::IllegalKeypointException iff <b>keypoint</b> is not recognized
     * @throws Extractor::IllegalDescriptorException iff <b>descriptor</b> is not recognized
     * @return
     */
    Extractor::Initializer::Ptr getInitializer(const std::string& keypoint, const std::string& descriptor);

    /**
     * @brief featureDetectors get a container of all detectors
     * @return
     */
    KeypointInitializerManager::Constructors featureDetectors() {
        return available_keypoints.availableClasses();
    }

    /**
     * @brief descriptorExtractors get a container of all extractors
     * @return
     */
    DescriptorInitializerManager::Constructors descriptorExtractors() {
        return available_descriptors.availableClasses();
    }

protected:
    struct ExtractorInitializerImp : public Extractor::Initializer {
        ExtractorInitializerImp(KeypointInitializer kc, DescriptorInitializer dc, bool complete)
            : kc_(kc), dc_(dc), complete_(complete)
        {}

        virtual void init(Extractor* e) {
            if(complete_) {
                kc_(e, true);
                // dc_ == kc_
            } else {
                kc_(e);
                dc_(e);
            }
        }
    private:
        KeypointInitializer kc_;
        DescriptorInitializer dc_;
        bool complete_;
    };

protected:
    KeypointInitializerManager available_keypoints;
    DescriptorInitializerManager available_descriptors;
};
}

#endif // EXTRACTOR_MANAGER_H
