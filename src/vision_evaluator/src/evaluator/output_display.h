#ifndef OUTPUT_DISPLAY_H
#define OUTPUT_DISPLAY_H

/// PROJECT
#include <designer/boxed_object.h>

/// SYSTEM
#include <QGraphicsView>

namespace vision_evaluator
{

class ConnectorIn;

class OutputDisplay : public BoxedObject
{
public:
    OutputDisplay();

    virtual void fill(QBoxLayout *layout);

private:
    ConnectorIn* input_;

    QGraphicsView* view_;
};

}

#endif // OUTPUT_DISPLAY_H
