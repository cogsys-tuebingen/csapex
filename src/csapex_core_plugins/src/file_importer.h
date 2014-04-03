#ifndef FILE_IMPORTER_H
#define FILE_IMPORTER_H

/// PROJECT
#include <csapex/model/message_provider.h>
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <QPushButton>

namespace csapex
{

class ConnectorOut;

class FileImporter : public BoxedObject
{
    Q_OBJECT

public:
    FileImporter();
    ~FileImporter();

    void setup();
    virtual void fill(QBoxLayout* layout);

    virtual QIcon getIcon() const;

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    void import(const QString& filename);

    void process();

    void tick();

    void importDialog();
    void toggle(bool on);
    bool doImport(const QString& path);

public:
    struct State : public Memento {
        State();

        typedef boost::shared_ptr<State> Ptr;

        FileImporter* parent;

        QString last_path_;

        Memento::Ptr sub_state;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

private:
    State state;

    MessageProvider::Ptr provider_;

    ConnectorIn* optional_input_filename_;

    ConnectorOut* output_;

    QHBoxLayout* additional_layout_;
    QPushButton* file_dialog_;

    QString to_import_;
};

}

#endif // FILE_IMPORTER_H
