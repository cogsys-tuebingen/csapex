#ifndef BOXED_OBJECT_H
#define BOXED_OBJECT_H

/// COMPONENT
#include <csapex/model/memento.h>
#include <csapex/view/displayable.h>
#include <csapex/model/tag.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <string>
#include <QLayout>
#include <QObject>
#include <QIcon>
#include <QMutex>

namespace csapex
{

class BoxedObject : public Displayable
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<BoxedObject> Ptr;
    static const Ptr NullPtr;

protected:
    BoxedObject();
    BoxedObject(const std::string& name);

public:
    virtual ~BoxedObject();

    void setName(const std::string& name);
    std::string getName();

    void setTypeName(const std::string& type_name);
    std::string getTypeName();

    void setCategory(const std::string& category) __attribute__ ((deprecated));

    void addTag(const Tag& tag);
    std::vector<Tag> getTags() const;

    void setIcon(QIcon icon);
    QIcon getIcon();

    virtual void setState(Memento::Ptr memento);
    virtual Memento::Ptr getState() const;

    virtual void fill(QBoxLayout* layout);

    virtual void updateDynamicGui(QBoxLayout* layout);
    virtual void updateModel();

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
    void guiChanged();

protected:
    void errorEvent(bool error, ErrorLevel level);

protected:
    std::string type_name_;
    std::string name_;

    mutable std::vector<Tag> tags_;

    QIcon icon_;

    bool enabled_;

    static int active_objects_;
    static QMutex active_mutex;
};



class NullBoxedObject : public BoxedObject {
    Q_OBJECT

public:
    virtual void fill(QBoxLayout*) {}

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn*) {}
};

}

#endif // BOXED_OBJECT_H
