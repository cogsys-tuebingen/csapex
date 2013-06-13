#ifndef FILE_IMPORTER_H
#define FILE_IMPORTER_H

/// COMPONENT
#include "image_provider.h"

/// PROJECT
#include <designer/boxed_object.h>

/// SYSTEM
#include <QPushButton>

namespace vision_evaluator
{

class ConnectorOut;

class FileImporter : public BoxedObject
{
    Q_OBJECT

public:
    FileImporter();

    virtual void fill(QBoxLayout *layout);

public Q_SLOTS:
    void importDialog();
    void publish();
    void toggle(bool on);

protected:
    void ticker();

private:
    ConnectorOut* output_img_;
    ConnectorOut* output_mask_;

    QPushButton *file_dialog_;

    QSharedPointer<ImageProvider> provider_;

    QTimer* timer_;
};

}

#endif // FILE_IMPORTER_H
