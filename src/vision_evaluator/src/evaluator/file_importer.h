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

class FileImporter;

class FileImporterWorker : public QObject
{
    Q_OBJECT

    friend class FileImporter;
    friend class State;

public:
    FileImporterWorker(FileImporter* parent);

public Q_SLOTS:
    void publish();
    bool import(const QString& path);

private:

    struct State : public Memento {
        State(FileImporter* parent)
            : parent(parent)
        {}

        FileImporter* parent;

        QString last_path_;

        Memento::Ptr sub_state;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;

    ImageProvider::Ptr provider_;

    QTimer* timer_;

    ConnectorOut* output_img_;
    ConnectorOut* output_mask_;
};

class FileImporter : public BoxedObject
{
    Q_OBJECT

    friend class FileImporterWorker;
    friend class FileImporterWorker::State;

public:
    FileImporter();
    ~FileImporter();

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    void import(const QString& filename);

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
