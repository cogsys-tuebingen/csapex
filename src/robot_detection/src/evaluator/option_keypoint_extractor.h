#ifndef OPTION_KEYPOINT_EXTRACTOR_H
#define OPTION_KEYPOINT_EXTRACTOR_H

/// COMPONENT
#include <csapex_vision/global_option.h>

/// PROJECT
#include <config/reconfigurable.h>

/// SYSTEM
#include <QComboBox>
#include <QSlider>

namespace robot_detection
{

class OptionKeypointExtractor : public csapex::GlobalOption, public Reconfigurable
{
    Q_OBJECT

public:
    OptionKeypointExtractor();

    ~OptionKeypointExtractor();

    virtual void insert(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

private Q_SLOTS:
    void update_type(int slot);
    void update_threshold(int t);

private:
    void configChanged();

private:
    QComboBox* selection;
    QSlider* threshold;
};

}

#endif // OPTION_KEYPOINT_EXTRACTOR_H
