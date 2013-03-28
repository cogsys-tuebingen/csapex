#ifndef OPTION_KEYPOINT_EXTRACTOR_H
#define OPTION_KEYPOINT_EXTRACTOR_H

/// COMPONENT
#include "option.h"

/// PROJECT
#include <config/reconfigurable.h>

/// SYSTEM
#include <QComboBox>
#include <QSlider>

class OptionKeypointExtractor : public Option, public Reconfigurable
{
    Q_OBJECT

public:
    OptionKeypointExtractor();

    ~OptionKeypointExtractor();

    virtual void insert(QLayout* layout);

public:
    static Option::TypePtr createInstance(CONSTRUCTOR_MODE mode);

private Q_SLOTS:
    void update_type(int slot);
    void update_threshold(int t);

private:
    void configChanged();

private:
    QComboBox* selection;
    QSlider* threshold;
};

#endif // OPTION_KEYPOINT_EXTRACTOR_H
