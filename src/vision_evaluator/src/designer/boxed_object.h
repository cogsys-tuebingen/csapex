#ifndef BOXED_OBJECT_H
#define BOXED_OBJECT_H

/// COMPONENT
#include "memento.h"

/// SYSTEM
#include <string>
#include <QLayout>
#include <QObject>
#include <QThread>

namespace vision_evaluator
{

class Box;

class BoxedObject : public QObject
{
    Q_OBJECT

public:
    BoxedObject();
    BoxedObject(const std::string& name);

    virtual ~BoxedObject();

    void setName(const std::string& name);
    std::string getName();

    void setBox(Box* box);

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr saveState();
    virtual void loadState(Memento::Ptr memento);

    bool isEnabled();

public Q_SLOTS:
    virtual void stop();

    virtual void enable(bool e);
    virtual void enable();
    virtual void disable(bool e);
    virtual void disable();
    virtual void connectorChanged();

protected:
    void makeThread();

protected:
    std::string name_;

    Box* box_;

    QThread* private_thread_;

    bool enabled_;
};

}

#endif // BOXED_OBJECT_H
