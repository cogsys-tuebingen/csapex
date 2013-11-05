/// HEADER
#include "extractor_manager.h"

/// COMPONENT
#include "extractor_factory.h"
#include <common/global.hpp>

/// SYSTEM
#include <algorithm>
#include <boost/foreach.hpp>

using namespace csapex;

ExtractorManager::ExtractorManager()
{
}

ExtractorManager::~ExtractorManager()
{
}

Extractor::Initializer::Ptr ExtractorManager::getInitializer(const std::string& keypoint, const std::string& descriptor)
{
    assert(!keypoint.empty() || !descriptor.empty());

    std::string keypoint_lower = keypoint;
    std::transform(keypoint_lower.begin(), keypoint_lower.end(), keypoint_lower.begin(), ::tolower);

    std::string descriptor_lower = descriptor;
    std::transform(descriptor_lower.begin(), descriptor_lower.end(), descriptor_lower.begin(), ::tolower);

    KeypointInitializer kc;
    DescriptorInitializer dc;

    if(!keypoint.empty()) {
        try {
            kc = available_keypoints.at(keypoint_lower);
        } catch(...) {
            FATAL("invalid keypoint type: '" << keypoint_lower << "'");
            throw Extractor::IllegalKeypointException();
        }
    }

    if(!descriptor.empty()) {
        try {
            dc = available_descriptors.at(descriptor_lower);
        } catch(...) {
            FATAL("invalid descriptor type: '" << descriptor_lower << "'");
            throw Extractor::IllegalDescriptorException();
        }
    }

    bool complete_set = (keypoint_lower == descriptor_lower) && (!keypoint.empty());

    Extractor::Initializer::Ptr i(new ExtractorInitializerImp(kc, dc, complete_set));

    return i;
}

void ExtractorManager::registerKeypointConstructor(const std::string& key, KeypointInit kc)
{
    std::string key_lower = key;
    std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);

    KeypointInitializer c;
    c.setType(key_lower);
    c.setConstructor(kc);
    available_keypoints[key_lower] = c;
}


void ExtractorManager::registerDescriptorConstructor(const std::string& key, DescriptorInit dc)
{
    std::string key_lower = key;
    std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);

    DescriptorInitializer c;
    c.setType(key_lower);
    c.setConstructor(dc);
    available_descriptors[key_lower] = c;
}


void ExtractorManager::registerKeypointParameters(const std::string& key, ParameterFunction f)
{
    std::string keypoint_lower = key;
    std::transform(keypoint_lower.begin(), keypoint_lower.end(), keypoint_lower.begin(), ::tolower);

    param_key[keypoint_lower] = f;
}

void ExtractorManager::registerDescriptorParameters(const std::string& des, ParameterFunction f)
{
    std::string descriptor_lower = des;
    std::transform(descriptor_lower.begin(), descriptor_lower.end(), descriptor_lower.begin(), ::tolower);

    param_des[descriptor_lower] = f;
}

std::vector<param::Parameter> ExtractorManager::featureDetectorParameters(const std::string& keypoint) {
    std::string keypoint_lower = keypoint;
    std::transform(keypoint_lower.begin(), keypoint_lower.end(), keypoint_lower.begin(), ::tolower);

    return param_key[keypoint_lower]();
}
std::vector<param::Parameter> ExtractorManager::featureDescriptorParameters(const std::string& descriptor) {
    std::string descriptor_lower = descriptor;
    std::transform(descriptor_lower.begin(), descriptor_lower.end(), descriptor_lower.begin(), ::tolower);

    return param_des[descriptor_lower]();
}
