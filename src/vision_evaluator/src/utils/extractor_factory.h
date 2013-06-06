#ifndef EXTRACTOR_FACTORY_H
#define EXTRACTOR_FACTORY_H

/// COMPONENT
#include "extractor.h"

/// PROJECT
#include <config/config.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>

namespace vision_evaluator
{
class ExtractorManager;
}

class ExtractorFactory
{
public:
    /**
     * @brief create instanciates a new Extractor
     * @param keypoint name of the keypoint detector
     * @param descriptor name of the descriptor extractor
     * @throws Extractor::IllegalKeypointException iff <b>keypoint</b> is not recognized
     * @throws Extractor::IllegalDescriptorException iff <b>descriptor</b> is not recognized
     * @return new instance
     */
    static Extractor::Ptr create(const std::string& keypoint, const std::string& descriptor);

private:
    ExtractorFactory();

    static ExtractorFactory& instance() {
        static ExtractorFactory i;
        return i;
    }

private:
    boost::shared_ptr<vision_evaluator::ExtractorManager> manager;
};

#endif // EXTRACTOR_FACTORY_H
