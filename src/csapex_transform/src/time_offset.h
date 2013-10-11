#ifndef TIME_OFFSET_H_
#define TIME_OFFSET_H_

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_core_plugins/ros_handler.h>
#include <csapex/utility/qdouble_slider.h>

namespace csapex {

class TimeOffset : public csapex::BoxedObject
{
    Q_OBJECT

public:
    TimeOffset();

    virtual void allConnectorsArrived();
    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    void update();

private:
    ConnectorOut* output_;
    ConnectorIn* input_;

    QDoubleSlider* offset_;

    struct State : public Memento {
        double offset_ms_;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;
};

}

#endif // TIME_OFFSET_H_
