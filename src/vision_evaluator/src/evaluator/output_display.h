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
    Q_OBJECT

public:
    OutputDisplay();
    virtual ~OutputDisplay();

    virtual void fill(QBoxLayout* layout);

    virtual void enable();
    virtual void disable();
    virtual void connectorChanged();

public Q_SLOTS:
    void messageArrived(ConnectorIn* source);

private:
    ConnectorIn* input_;
    QPixmap pixmap_;

    QGraphicsView* view_;

    QImage empty;
    QPainter painter;

    bool display_is_empty;
};

}

#endif // OUTPUT_DISPLAY_H
