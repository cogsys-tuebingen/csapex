/// HEADER
#include "connector.h"

/// COMPONENT
#include "design_board.h"
#include "box.h"

/// SYSTEM
#include <iostream>
#include <typeinfo>

Connector::Connector(QWidget* parent)
    : DragTracker(this), parent_widget(parent), box(NULL), designer(NULL)
{
    findParents();
    setFocusPolicy(Qt::NoFocus);
}

Connector::~Connector()
{
//    overlay_->connectorRemoved(this);
}

bool Connector::tryConnect(QObject *other_side)
{
    Connector* c = dynamic_cast<Connector*> (other_side);
    if(c) {
        return tryConnect(c);
    }
    return false;
}

void Connector::removeConnection(QObject *other_side)
{
    Connector* c = dynamic_cast<Connector*> (other_side);
    if(c) {
        removeConnection(c);
    }
}

void Connector::findParents()
{
    QWidget * tmp = this;
    while(tmp != NULL) {
        if(dynamic_cast<vision_evaluator::Box*>(tmp)) {
            box = dynamic_cast<vision_evaluator::Box*>(tmp);
        } else if(dynamic_cast<vision_evaluator::DesignBoard*>(tmp)) {
            designer = dynamic_cast<vision_evaluator::DesignBoard*>(tmp);
        }
        tmp = tmp->parentWidget();
    }
}

bool Connector::hitButton(const QPoint &) const
{
    return false;
}

void Connector::mousePressEvent(QMouseEvent *e)
{
    DragTracker::mousePressEvent(e);
}

void Connector::mouseReleaseEvent(QMouseEvent *e)
{
    DragTracker::mouseReleaseEvent(e);

    e->accept();

    overlay_->deleteTemporaryLine();
    designer->connectorReleased(this, e->globalPos());
}

QPoint Connector::topLeft()
{
    if(box == NULL) {
        findParents();
    }

    return box->geometry().topLeft() + /*box->widget()->geometry().topLeft() + */ pos();
}

QPoint Connector::centerPoint()
{
    return topLeft() + 0.5 * (geometry().bottomRight() - geometry().topLeft());
}

void Connector::mouseDelta(const QPoint& delta)
{
    QPoint radio_center = centerPoint();

    if(isOutput()) {
        overlay_->drawTemporaryLine(QLine(radio_center, radio_center + delta));
    } else {
        overlay_->drawTemporaryLine(QLine(radio_center + delta, radio_center));
    }
}

QPoint Connector::getPos() const
{
    return pos();
}

void Connector::paintEvent(QPaintEvent *e)
{
    setAutoExclusive(false);
    setChecked(isConnected());

    QRadioButton::paintEvent(e);
}
