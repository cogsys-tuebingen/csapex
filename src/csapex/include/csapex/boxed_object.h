#ifndef BOXED_OBJECT_H
#define BOXED_OBJECT_H

/// COMPONENT
#include "memento.h"
#include "displayable.h"

/// SYSTEM
#include <string>
#include <QLayout>
#include <QObject>
#include <QIcon>

namespace csapex
{

class Box;
class ConnectorIn;
class ConnectorOut;

class BoxedObject : public Displayable
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

    void setIcon(QIcon icon);
    QIcon getIcon();

    virtual void setState(Memento::Ptr memento);
    virtual Memento::Ptr getState() const;

    virtual void fill(QBoxLayout* layout);
    virtual void updateDynamicGui(QBoxLayout* layout);

    virtual bool canBeDisabled() const;

    bool isEnabled();

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source) = 0;

    virtual void enable(bool e);
    virtual void enable();
    virtual void disable(bool e);
    virtual void disable();
    virtual void connectorChanged();

    virtual void tick();

Q_SIGNALS:
    void modelChanged();

protected:
    void errorEvent(bool error, ErrorLevel level);

protected:
    std::string type_name_;
    std::string name_;
    std::string category_;

    QIcon icon_;

    bool enabled_;
};

}

#endif // BOXED_OBJECT_H
