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

class FileImporterWorker : public QObject
{
    Q_OBJECT

    friend class FileImporter;

public:
    FileImporterWorker();

public Q_SLOTS:
    void publish();
    bool import(const QString& path);

private:
    QTimer* timer_;

    ConnectorOut* output_img_;
    ConnectorOut* output_mask_;

    QString last_path_;
    QSharedPointer<ImageProvider> provider_;
};

class FileImporter : public BoxedObject
{
    Q_OBJECT

public:
    FileImporter();
    ~FileImporter();

    virtual void fill(QBoxLayout* layout);

public Q_SLOTS:
    void importDialog();
    void toggle(bool on);

private:
    FileImporterWorker* worker;

    QHBoxLayout* additional_layout_;
    QPushButton* file_dialog_;
};

}

#endif // FILE_IMPORTER_H
