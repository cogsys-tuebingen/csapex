#ifndef BOXED_OBJECT_H
#define BOXED_OBJECT_H

/// COMPONENT
#include "memento.h"

/// SYSTEM
#include <string>
#include <QLayout>
#include <QObject>

namespace vision_evaluator
{

class Box;
class ConnectorIn;
class ConnectorOut;

class BoxedObject : public QObject
{
    Q_OBJECT

public:
    BoxedObject();
    BoxedObject(const std::string& name);

    virtual ~BoxedObject();

    void setName(const std::string& name);
    std::string getName();

    void setTypeName(const std::string& type_name);
    std::string getTypeName();

    void setCategory(const std::string& category);
    std::string getCategory();

    void setBox(Box* box);

    virtual void fill(QBoxLayout* layout);
    virtual void updateDynamicGui(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    virtual bool canBeDisabled() const;

    bool isEnabled();
    bool isError();

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source) = 0;

    virtual void setError(bool e, const std::string& msg = "");

    virtual void enable(bool e);
    virtual void enable();
    virtual void disable(bool e);
    virtual void disable();
    virtual void connectorChanged();

    virtual void tick();

Q_SIGNALS:
    void modelChanged();

protected:
    std::string type_name_;
    std::string name_;
    std::string category_;

    Box* box_;

    bool enabled_;
    bool error_;
};

}

#endif // BOXED_OBJECT_H
