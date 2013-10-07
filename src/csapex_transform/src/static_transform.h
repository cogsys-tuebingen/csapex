#ifndef STATIC_TRANSFORM_H
#define STATIC_TRANSFORM_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex/utility/qdouble_slider.h>

namespace csapex {

class StaticTransform : public csapex::BoxedObject
{
    Q_OBJECT

public:
    StaticTransform();

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    virtual void tick();
    void update();

private:
    ConnectorOut* output_;

    QDoubleSlider* roll_;
    QDoubleSlider* pitch_;
    QDoubleSlider* yaw_;

    QDoubleSlider* x_;
    QDoubleSlider* y_;
    QDoubleSlider* z_;

    struct State : public Memento {
        double roll,pitch,yaw,x,y,z;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;
};

}

#endif // STATIC_TRANSFORM_H
