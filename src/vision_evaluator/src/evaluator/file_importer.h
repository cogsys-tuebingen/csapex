#ifndef FILE_IMPORTER_H
#define FILE_IMPORTER_H

/// PROJECT
#include <designer/boxed_object.h>

namespace vision_evaluator
{

class ConnectorOut;

class FileImporter : public BoxedObject
{
public:
    FileImporter();

    virtual void fill(QBoxLayout *layout);

private:
    ConnectorOut* output_;
};

}

#endif // FILE_IMPORTER_H
