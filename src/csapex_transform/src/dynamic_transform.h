#ifndef DYNAMIC_TRANSFORM_H
#define DYNAMIC_TRANSFORM_H

/// PROJECT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <ros/time.h>
#include <QComboBox>
#include <QPushButton>

namespace csapex {

class DynamicTransform : public csapex::BoxedObject
{
    Q_OBJECT

public:
    DynamicTransform();

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);


public Q_SLOTS:
    virtual void process();
    virtual void tick();
    void update();
    void updateFrames();
    void resetTf();

private:
    void publishTransform(const ros::Time& time);

private:
    ConnectorOut* output_;
    ConnectorOut* output_frame_;

    ConnectorIn* frame_in_from_;
    ConnectorIn* frame_in_to_;
    ConnectorIn* time_in_;

    QComboBox* from_box_;
    QComboBox* to_box_;

    QPushButton* refresh_;
    QPushButton* reset_tf_;

    struct State : public Memento {
        std::string from_;
        std::string to_;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;
};

}

#endif // DYNAMIC_TRANSFORM_H
