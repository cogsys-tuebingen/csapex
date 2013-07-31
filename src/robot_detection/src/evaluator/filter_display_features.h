#ifndef FILTERDISPLAYFEATURES_H
#define FILTERDISPLAYFEATURES_H

/// COMPONENT
#include <csapex_vision/filter.h>

/// PROJECT
#include <config/reconfigurable.h>

/// SYSTEM
#include <QCheckBox>
#include <QComboBox>

namespace robot_detection
{

class FilterDisplayFeatures : public csapex::Filter, public Reconfigurable
{
    Q_OBJECT

public:
    FilterDisplayFeatures();

public:
    virtual void filter(cv::Mat& img, cv::Mat& mask);
    virtual void insert(QBoxLayout* layout);

private Q_SLOTS:
    void update(int slot);
    void update();

private:
    cv::Scalar color;
    int flags;

    QComboBox* colorbox;
    QCheckBox* richbox;
};

}

#endif // FILTERDISPLAYFEATURES_H
