#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/tag.h>
#include <csapex/model/memento.h>
#include <csapex/view/displayable.h>

/// SYSTEM
#include <QObject>
#include <QIcon>

namespace csapex {

class Node : public QObject, public Displayable
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<Node> Ptr;

public:
    Node();
    virtual ~Node();

    void setType(const std::string& type);
    std::string getType();

    void setCategory(const std::string& category) __attribute__ ((deprecated));

    void addTag(const Tag& tag);
    std::vector<Tag> getTags() const;

    void setIcon(QIcon icon);
    QIcon getIcon();


    virtual bool canBeDisabled() const;
    bool isEnabled();

    virtual void setState(Memento::Ptr memento);
    virtual Memento::Ptr getState() const;


public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source);
    virtual void allConnectorsArrived();

    virtual void enable(bool e);
    virtual void enable();
    virtual void disable(bool e);
    virtual void disable();
    virtual void connectorChanged();

    virtual void tick();

    virtual void updateModel();

protected:
    std::string type_;

    mutable std::vector<Tag> tags_;

    QIcon icon_;

    bool enabled_;
};

}

#endif // NODE_H_
