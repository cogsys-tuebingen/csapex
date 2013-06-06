/// HEADER
#include "extractor_factory.h"

/// COMPONENT
#include "extractor_manager.h"

ExtractorFactory::ExtractorFactory()
    : manager(new vision_evaluator::ExtractorManager)
{
}

Extractor::Ptr ExtractorFactory::create(const std::string& keypoint, const std::string& descriptor)
{
    Extractor::Ptr e(new Extractor);

    instance().manager->getInitializer(keypoint, descriptor)->init(e.get());

    return e;
}
