#ifndef TEXT_DISPLAY_H_
#define TEXT_DISPLAY_H_

/// PROJECT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <QLabel>

namespace csapex {

class SayText : public Node
{
public:
    SayText();

    virtual void process();
    virtual void setup();

private:
    ConnectorIn* connector_;
};

}

#endif // TEXT_DISPLAY_H_
