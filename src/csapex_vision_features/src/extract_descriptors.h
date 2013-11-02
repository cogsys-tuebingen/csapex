#ifndef EXTRACT_DESCRIPTORS_H
#define EXTRACT_DESCRIPTORS_H

/// COMPONENT
#include <csapex/model/boxed_object.h>

/// PROJECT
#include <utils/extractor.h>

/// SYSTEM
#include <QCheckBox>
#include <QComboBox>
#include <QMutex>

namespace csapex
{

class ExtractDescriptors : public csapex::BoxedObject
{
    Q_OBJECT

public:
    ExtractDescriptors();

public:
    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    virtual void allConnectorsArrived();
    void updateDynamicGui(QBoxLayout *layout);

private Q_SLOTS:
    void update(int slot);
    void update();
    void updateModel();

private:
    template <typename T>
    void updateParam(const std::string& name, T value);

    // TODO: state
private:
    QComboBox* selection_des;

    bool change;

    std::vector<QObject*> callbacks;

    QFrame* opt;

    QMutex extractor_mutex;
    Extractor::Ptr extractor;

    ConnectorIn* in_img;
    ConnectorIn* in_key;
    ConnectorOut* out_des;

    struct State : public Memento {
        std::string des;
        std::map<std::string, std::vector<param::Parameter> > params;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;
};

}

#endif // EXTRACT_DESCRIPTORS_H
