#ifndef OPTION_KEYPOINT_DESCRIPTOR_H
#define OPTION_KEYPOINT_DESCRIPTOR_H

/// COMPONENT
#include "option.h"

/// PROJECT
#include <config/reconfigurable.h>

/// SYSTEM
#include <QComboBox>

class OptionKeypointDescriptor : public Option, public Reconfigurable
{
    Q_OBJECT

public:
    OptionKeypointDescriptor();

    ~OptionKeypointDescriptor();

    virtual void insert(QLayout* layout);

public:
    static TypePtr createInstance(CONSTRUCTOR_MODE mode);

private Q_SLOTS:
    void update(int slot);

private:
    void configChanged();

private:
    QComboBox* selection;
};

#endif // OPTION_KEYPOINT_DESCRIPTOR_H
