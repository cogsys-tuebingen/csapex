#ifndef TEXT_DISPLAY_H_
#define TEXT_DISPLAY_H_

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <QLabel>

namespace csapex {

class SayText : public Node
{
public:
    SayText();

    virtual void process();
    virtual void setup();

    virtual QIcon getIcon() const;

private:
    ConnectorIn* connector_;
};

}

#endif // TEXT_DISPLAY_H_
