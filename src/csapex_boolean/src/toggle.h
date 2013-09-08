#ifndef TOGGLE_H
#define TOGGLE_H

/// HEADER
#include <csapex/boxed_object.h>

/// SYSTEM
#include <QPushButton>

namespace csapex {

namespace boolean {

class Toggle : public BoxedObject
{
    Q_OBJECT

public:
    Toggle();

public:
    virtual void fill(QBoxLayout* layout);

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source);
    void setSignal(bool signal);

    void tick();

private:
    ConnectorOut* out;

    bool signal_;
    QPushButton* btn;
};

}

}

#endif // TOGGLE_H
