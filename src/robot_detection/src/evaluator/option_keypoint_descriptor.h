#ifndef OPTION_KEYPOINT_DESCRIPTOR_H
#define OPTION_KEYPOINT_DESCRIPTOR_H

/// COMPONENT
#include <vision_evaluator/global_option.h>

/// PROJECT
#include <config/reconfigurable.h>

/// SYSTEM
#include <QComboBox>

namespace robot_detection
{

class OptionKeypointDescriptor : public vision_evaluator::GlobalOption, public Reconfigurable
{
    Q_OBJECT

public:
    OptionKeypointDescriptor();

    ~OptionKeypointDescriptor();

    virtual void insert(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

private Q_SLOTS:
    void update(int slot);

private:
    void configChanged();

private:
    QComboBox* selection;
};

}

#endif // OPTION_KEYPOINT_DESCRIPTOR_H
