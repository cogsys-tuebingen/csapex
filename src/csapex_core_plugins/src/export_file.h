#ifndef EXPORT_FILE_H_
#define EXPORT_FILE_H_

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex/model/connection_type.h>

/// SYSTEM
#include <QMutex>
#include <QPushButton>

namespace csapex {

class ExportFile : public BoxedObject
{
    Q_OBJECT

public:
    ExportFile();

    virtual void fill(QBoxLayout* layout);
    virtual void allConnectorsArrived();

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    void setExportPath(const QString& path);

public Q_SLOTS:
    void exportDialog();

private:
    ConnectorIn* connector_;

    QMutex mutex_;

    struct State : public Memento {
        std::string path_;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    int suffix_;

    State state;

    QHBoxLayout* additional_layout_;
    QPushButton* file_dialog_;
};

}

#endif // EXPORT_FILE_H_
