#ifndef FILTERDISPLAYFEATURES_H
#define FILTERDISPLAYFEATURES_H

/// COMPONENT
#include "filter.h"
#include "option.h"

/// PROJECT
#include <config/reconfigurable.h>

/// SYSTEM
#include <QCheckBox>

class FilterDisplayFeaturesOption : public Option
{
    Q_OBJECT

    FilterDisplayFeaturesOption();
    virtual void insert(QLayout* layout);

public:
    static Option::TypePtr createInstance(CONSTRUCTOR_MODE mode);

private Q_SLOTS:
    void update(int slot);
    void update();

public:
    cv::Scalar color;
    int flags;

private:
    QComboBox* colorbox;
    QCheckBox* richbox;
};

class FilterDisplayFeatures : public Filter, public Reconfigurable
{
    Q_OBJECT

private:
    FilterDisplayFeatures();

public:
    static Filter::TypePtr createInstance(CONSTRUCTOR_MODE mode);

    virtual void filter(cv::Mat img, cv::Mat mask);

    virtual void insert(QLayout*);
};

#endif // FILTERDISPLAYFEATURES_H
