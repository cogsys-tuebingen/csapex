#ifndef BOXED_OBJECT_H
#define BOXED_OBJECT_H

/// SYSTEM
#include <string>
#include <QLayout>

namespace vision_evaluator {

class Box;

class BoxedObject
{
public:
    BoxedObject();
    BoxedObject(const std::string& name);

    virtual ~BoxedObject();

    void setName(const std::string& name);
    std::string getName();

    void setBox(Box* box);

    virtual void fill(QBoxLayout* layout);

protected:
    std::string name_;
    Box* box_;
};

}

#endif // BOXED_OBJECT_H
