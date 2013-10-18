#ifndef TEXT_DISPLAY_H_
#define TEXT_DISPLAY_H_

/// PROJECT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <QLabel>

namespace csapex {

class SayText : public BoxedObject
{
    Q_OBJECT

public:
    SayText();

    virtual void allConnectorsArrived();
    virtual void fill(QBoxLayout* layout);

private:
    ConnectorIn* connector_;
};

}

#endif // TEXT_DISPLAY_H_
