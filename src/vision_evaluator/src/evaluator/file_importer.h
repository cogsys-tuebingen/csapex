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
    ~FileImporter();

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    void import(const QString& filename);

public Q_SLOTS:
    void messageArrived(ConnectorIn* source);
    void tick();

    void importDialog();
    void toggle(bool on);
    bool doImport(const QString& path);
    void enableBorder(int border);

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

    ConnectorIn* optional_input_filename_;

    ConnectorOut* output_img_;
    ConnectorOut* output_mask_;

    QHBoxLayout* additional_layout_;
    QPushButton* file_dialog_;
};

}

#endif // FILE_IMPORTER_H
