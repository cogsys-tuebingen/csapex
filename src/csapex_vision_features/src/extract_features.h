#ifndef EXTRACT_FEATURES_H
#define EXTRACT_FEATURES_H

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

class ExtractFeatures : public csapex::BoxedObject
{
    Q_OBJECT

public:
    ExtractFeatures();

public:
    virtual void fill(QBoxLayout* layout);

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source);

private Q_SLOTS:
    void update(int slot);
    void update();

    // TODO: state
private:
    QComboBox* selection_key;
    QComboBox* selection_des;

    QMutex extractor_mutex;
    Extractor::Ptr extractor;

    ConnectorIn* in_img;
    ConnectorIn* in_mask;
    ConnectorOut* out_key;
    ConnectorOut* out_des;

    bool has_img;
    bool has_mask;
};

}

#endif // EXTRACT_FEATURES_H
