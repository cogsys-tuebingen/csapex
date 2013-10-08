#ifndef DYNAMIC_TRANSFORM_H
#define DYNAMIC_TRANSFORM_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_core_plugins/ros_handler.h>

/// SYSTEM
#include <QComboBox>
#include <QPushButton>
#include <tf/transform_listener.h>

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
    virtual void allConnectorsArrived();
    virtual void tick();
    void update();
    void updateFrames();

private:
    void publishTransform(const ros::Time& time);

    struct Listener {
        tf::TransformListener tfl;

        static Listener& instance() {
            ROSHandler::instance().waitForConnection();
            static Listener l; return l; }
    private:
        Listener() {}
    };

private:
    ConnectorOut* output_;
    ConnectorIn* time_in_;

    QComboBox* from_box_;
    QComboBox* to_box_;

    QPushButton* refresh_;

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
