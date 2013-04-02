#ifndef EXTRACTOR_FACTORY_H
#define EXTRACTOR_FACTORY_H

/// COMPONENT
#include "extractor.h"

/// PROJECT
#include <config/config.h>

class ExtractorFactory
{
private:
    ExtractorFactory();

public:
    static Extractor::Ptr create(Types::Keypoint::ID keypoint, Types::Descriptor::ID descriptor);

    static void init_keypoint(Extractor::Ptr& e, Types::Keypoint::ID keypoint);
    static void init_descriptor(Extractor::Ptr& e, Types::Descriptor::ID descriptor);
};

#endif // EXTRACTOR_FACTORY_H
