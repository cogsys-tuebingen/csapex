#ifndef OUTPUT_DISPLAY_H
#define OUTPUT_DISPLAY_H

/// PROJECT
#include <csapex/model/boxed_object.h>

namespace csapex
{

class ConnectorIn;

class OutputDisplay : public Node
{
    friend class OutputDisplayAdapter;

public:
    OutputDisplay();
    virtual ~OutputDisplay();

    void setup();
    void process();

//    virtual void enable();
//    virtual void disable();
    virtual void connectorChanged();

protected:
    ConnectorIn* input_;

public:
    boost::signals2::signal<void(QSharedPointer<QImage>)> display_request;
};

}

#endif // OUTPUT_DISPLAY_H
