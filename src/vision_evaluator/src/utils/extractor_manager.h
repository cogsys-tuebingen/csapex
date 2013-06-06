#ifndef EXTRACTOR_MANAGER_H
#define EXTRACTOR_MANAGER_H

/// COMPONENT
#include "extractor.h"

/// PROJECT
#include <evaluator/generic_manager.hpp>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <boost/function.hpp>
#include <boost/foreach.hpp>

#define EXTRACTOR_IMPLEMENTATION \
    std::string getName() { return name; }\
    static const std::string name;

#define REGISTER_FEATURE_DETECTOR(class_name, impname)\
    const std::string class_name::name(#impname); \
    STATIC_INIT(keypoint_##class_name, class_name, { \
        boost::shared_ptr<class_name> tmp(new class_name); \
        ExtractorManager manager;\
        if(boost::dynamic_pointer_cast<ExtractorManager::FeatureDetectorConstructor>(tmp)) {\
            manager.registerKeypointConstructor(class_name::name, boost::dynamic_pointer_cast<ExtractorManager::FeatureDetectorConstructor>(tmp)); \
        } \
        if(boost::dynamic_pointer_cast<ExtractorManager::DescriptorConstructor>(tmp)) {\
            manager.registerDescriptorConstructor(class_name::name, boost::dynamic_pointer_cast<ExtractorManager::DescriptorConstructor>(tmp)); \
        } \
    })\
 

namespace vision_evaluator
{
/**
 * @brief The ExtractorManager class manages all instances of feature detectors and descriptor extractors.
 *        It functions like a proxy to the underlying singleton classes.
 */
class ExtractorManager
{
public:
    /**
     * @brief The FeatureDetectorConstructor interface
     */
    struct FeatureDetectorConstructor {
        typedef boost::shared_ptr<FeatureDetectorConstructor> Ptr;
        virtual void keypoint(Extractor* e, bool complete = false) = 0;
        virtual bool valid() {
            return true;
        }
        virtual std::string getName() = 0;
    };

    /**
     * @brief The DescriptorConstructor interface
     */
    struct DescriptorConstructor {
        typedef boost::shared_ptr<DescriptorConstructor> Ptr;
        virtual void descriptor(Extractor* e) = 0;
        virtual bool valid() {
            return true;
        }
        virtual std::string getName() = 0;
    };

    struct CompleteExtractorConstructor
            : public FeatureDetectorConstructor, public DescriptorConstructor {
    };

    typedef PluginManager<FeatureDetectorConstructor, FeatureDetectorConstructor::Ptr, std::map<std::string, FeatureDetectorConstructor::Ptr> > FeatureDetectorManager;
    typedef PluginManager<DescriptorConstructor, DescriptorConstructor::Ptr, std::map<std::string, DescriptorConstructor::Ptr> > DescriptorManager;

public:
    /**
     * @brief ExtractorManager functions like a proxy to the underlying singleton classes.
     */
    ExtractorManager();

    /**
     * @brief ~ExtractorManager
     */
    virtual ~ExtractorManager();

    /**
     * @brief registerKeypointConstructor installs a new keypoint detector instance
     * @param key name of the instance, must be unique
     * @param kc Constructor of the instance, called if <b>key</b> is requested
     */
    void registerKeypointConstructor(const std::string& key, FeatureDetectorConstructor::Ptr kc);

    /**
     * @brief registerDescriptorConstructor installs a new descriptor extractor instance
     * @param key name of the instance, must be unique
     * @param dc Constructor of the instance, called if <b>key</b> is requested
     */
    void registerDescriptorConstructor(const std::string& key, DescriptorConstructor::Ptr dc);

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
    typename FeatureDetectorManager::Constructors featureDetectors() {
        return available_keypoints.availableClasses();
    }

    /**
     * @brief descriptorExtractors get a container of all extractors
     * @return
     */
    typename DescriptorManager::Constructors descriptorExtractors() {
        return available_descriptors.availableClasses();
    }

protected:
    struct ExtractorInitializerImp : public Extractor::Initializer {
        ExtractorInitializerImp(FeatureDetectorConstructor::Ptr kc, DescriptorConstructor::Ptr dc, bool complete)
            : kc_(kc), dc_(dc), complete_(complete)
        {}

        virtual void init(Extractor* e) {
            assert(kc_.get());
            assert(dc_.get());
            if(complete_) {
                kc_->keypoint(e, true);
                // dc_ == kc_
            } else {
                kc_->keypoint(e);
                dc_->descriptor(e);
            }
        }
    private:
        FeatureDetectorConstructor::Ptr kc_;
        DescriptorConstructor::Ptr dc_;
        bool complete_;
    };

protected:
    FeatureDetectorManager available_keypoints;
    DescriptorManager available_descriptors;
};
}

#endif // EXTRACTOR_MANAGER_H
