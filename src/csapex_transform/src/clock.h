#ifndef CLOCK_H
#define CLOCK_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_core_plugins/ros_handler.h>

/// SYSTEM
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <tf/transform_listener.h>

namespace csapex {

class Clock : public csapex::BoxedObject
{
    Q_OBJECT

public:
    Clock();

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    virtual void tick();
    void update();

private:
    ConnectorOut* output_;

    QPushButton* time_type_;
    QLabel* time_label_;

    struct State : public Memento {
        bool use_ros_time;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;
};

}

#endif // CLOCK_H
