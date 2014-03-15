#ifndef DOUBLE_INPUT_H
#define DOUBLE_INPUT_H

/// PROJECT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <QDoubleSpinBox>

namespace csapex {

class DoubleInput : public BoxedObject
{
    Q_OBJECT

public:
    DoubleInput();

    void process();
    virtual void fill(QBoxLayout* layout);

public Q_SLOTS:
    void setValue(double v);
    void publish();

private:
    ConnectorOut* connector_;

    struct State : public Memento {
        double message;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    State state;

    QDoubleSpinBox* sbox_;
};

}

#endif // DOUBLE_INPUT_H
