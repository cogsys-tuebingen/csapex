/// HEADER
#include "extractor_manager.h"

/// COMPONENT
#include "extractor_factory.h"

/// SYSTEM
#include <algorithm>
#include <boost/foreach.hpp>

using namespace vision_evaluator;

ExtractorManager::ExtractorManager()
    : available_keypoints("vision_evaluator::ExtractorManager::KeypointInitializerManager"),
      available_descriptors("vision_evaluator::ExtractorManager::DescriptorInitializerManager")
{
}

ExtractorManager::~ExtractorManager()
{
}

Extractor::Initializer::Ptr ExtractorManager::getInitializer(const std::string& keypoint, const std::string& descriptor)
{
    std::string keypoint_lower = keypoint;
    std::transform(keypoint_lower.begin(), keypoint_lower.end(), keypoint_lower.begin(), ::tolower);

    std::string descriptor_lower = descriptor;
    std::transform(descriptor_lower.begin(), descriptor_lower.end(), descriptor_lower.begin(), ::tolower);

    KeypointInitializerManager::Constructor kc = available_keypoints.availableClasses(keypoint_lower);
    DescriptorInitializerManager::Constructor dc = available_descriptors.availableClasses(descriptor_lower);

    if(kc.valid())
        FATAL("invalid keypoint type: '" << keypoint_lower << "'");
    if(dc.valid())
        FATAL("invalid descriptor type: '" << descriptor_lower << "'");

    if(kc.valid())
        throw Extractor::IllegalKeypointException();
    if(dc.valid())
        throw Extractor::IllegalDescriptorException();

    bool complete_set = keypoint_lower == descriptor_lower;

    Extractor::Initializer::Ptr i(new ExtractorInitializerImp(kc, dc, complete_set));

    return i;
}

void ExtractorManager::registerKeypointConstructor(const std::string& key, KeypointInit kc)
{
    std::string key_lower = key;
    std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);
    std::cout << "register keypoint detector instance " << key_lower << std::endl;

    KeypointInitializer c;
    c.setType(key_lower);
    c.setConstructor(kc);
    available_keypoints.availableClasses(key_lower) = c;
}


void ExtractorManager::registerDescriptorConstructor(const std::string& key, DescriptorInit dc)
{
    std::string key_lower = key;
    std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);
    std::cout << "register descriptor extractor instance " << key_lower << std::endl;

    DescriptorInitializer c;
    c.setType(key_lower);
    c.setConstructor(dc);
    available_descriptors.availableClasses(key_lower) = c;
}
