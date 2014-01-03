#ifndef INT_INPUT_H
#define INT_INPUT_H

/// PROJECT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <QSpinBox>

namespace csapex {

class IntInput : public BoxedObject
{
    Q_OBJECT

public:
    IntInput();

    virtual void fill(QBoxLayout* layout);

public Q_SLOTS:
    void setValue(int v);
    void publish();

private:
    ConnectorOut* connector_;

    struct State : public Memento {
        int message;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    State state;

    QSpinBox* sbox_;
};

}

#endif // INT_INPUT_H
