#ifndef EXTRACT_DESCRIPTORS_H
#define EXTRACT_DESCRIPTORS_H

/// COMPONENT
#include <csapex/boxed_object.h>

/// PROJECT
#include <config/reconfigurable.h>
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

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source);

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

    std::string des;
    std::vector<vision::Parameter> params;
    std::vector<QObject*> callbacks;

    QFrame* opt;

    QMutex extractor_mutex;
    Extractor::Ptr extractor;

    ConnectorIn* in_img;
    ConnectorIn* in_key;
    ConnectorOut* out_des;

    bool has_img;
    bool has_kp;
};

}

#endif // EXTRACT_DESCRIPTORS_H
