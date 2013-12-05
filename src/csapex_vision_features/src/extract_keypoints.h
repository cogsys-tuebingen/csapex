#ifndef EXTRACT_FEATURES_H
#define EXTRACT_FEATURES_H

/// COMPONENT
#include <csapex/model/boxed_object.h>

/// PROJECT
#include <config/reconfigurable.h>
#include <utils/extractor.h>

/// SYSTEM
#include <QCheckBox>
#include <QComboBox>
#include <QMutex>

namespace csapex
{

class ExtractKeypoints : public csapex::BoxedObject
{
    Q_OBJECT

public:
    ExtractKeypoints();

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
    QComboBox* selection_key;

    bool change;

    std::vector<QObject*> callbacks;

    QFrame* opt;

    QMutex extractor_mutex;
    Extractor::Ptr extractor;

    ConnectorIn* in_img;
    ConnectorIn* in_mask;
    ConnectorOut* out_key;

    struct State : public Memento {
        std::string key;
        std::map<std::string, std::vector<param::Parameter::Ptr> > params;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;

};

}

#endif // EXTRACT_FEATURES_H
