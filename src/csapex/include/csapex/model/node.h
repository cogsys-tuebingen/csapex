#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/tag.h>

/// SYSTEM
#include <QObject>
#include <QIcon>

namespace csapex {

class Node : public QObject
{
    Q_OBJECT

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

protected:
    std::string type_;

    mutable std::vector<Tag> tags_;

    QIcon icon_;
};

}

#endif // NODE_H_
