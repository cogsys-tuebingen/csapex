#ifndef TEXT_INPUT_H
#define TEXT_INPUT_H

/// PROJECT
#include <csapex/boxed_object.h>

/// SYSTEM
#include <QLineEdit>

namespace csapex {

class TextInput : public BoxedObject
{
    Q_OBJECT

public:
    TextInput();

    virtual void fill(QBoxLayout* layout);
    virtual void messageArrived(ConnectorIn* source);

public Q_SLOTS:
    void setText(QString text);
    void publish();

private:
    ConnectorOut* connector_;

    struct State : public Memento {
        std::string message;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    State state;

    QLineEdit* txt_;
};

}

#endif // TEXT_INPUT_H
