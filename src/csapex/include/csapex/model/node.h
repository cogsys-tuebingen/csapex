#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QObject>

namespace csapex {

class Node : public QObject
{
    Q_OBJECT

public:
    virtual ~Node();
};

}

#endif // NODE_H_
