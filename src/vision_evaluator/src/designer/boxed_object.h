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
public:
    BoxedObject();
    BoxedObject(const std::string& name);

    virtual ~BoxedObject();

    virtual void stop();

    void setName(const std::string& name);
    std::string getName();

    void setBox(Box* box);

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr saveState();
    virtual void loadState(Memento::Ptr memento);

protected:
    void makeThread();

protected:
    std::string name_;

    Box* box_;

    QThread* private_thread_;
};

}

#endif // BOXED_OBJECT_H
