/// HEADER
#include "connector.h"

/// COMPONENT
#include "design_board.h"
#include "box.h"

/// SYSTEM
#include <iostream>
#include <typeinfo>
#include <QDragEnterEvent>

using namespace vision_evaluator;

const QString Connector::MIME = "vision_evaluator/connector";

Connector::Connector(QWidget* parent)
    : QRadioButton(parent), parent_widget(parent), box(NULL), designer(NULL)
{
    findParents();
    setFocusPolicy(Qt::NoFocus);
    setAcceptDrops(true);
}

Connector::~Connector()
{
}

bool Connector::tryConnect(QObject* other_side)
{
    Connector* c = dynamic_cast<Connector*>(other_side);
    if(c) {
        return tryConnect(c);
    }
    return false;
}

void Connector::removeConnection(QObject* other_side)
{
    Connector* c = dynamic_cast<Connector*>(other_side);
    if(c) {
        removeConnection(c);
    }
}

void Connector::findParents()
{
    QWidget* tmp = this;
    while(tmp != NULL) {
        if(dynamic_cast<vision_evaluator::Box*>(tmp)) {
            box = dynamic_cast<vision_evaluator::Box*>(tmp);
        } else if(dynamic_cast<vision_evaluator::DesignBoard*>(tmp)) {
            designer = dynamic_cast<vision_evaluator::DesignBoard*>(tmp);
        }
        tmp = tmp->parentWidget();
    }
}

bool Connector::hitButton(const QPoint&) const
{
    return false;
}

bool Connector::canConnectTo(Connector* other_side)
{
    return (isOutput() && other_side->isInput()) || (isInput() && other_side->isOutput());
}

void Connector::dragEnterEvent(QDragEnterEvent* e)
{
    if(e->mimeData()->text() == Connector::MIME) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());
        if(from->canConnectTo(this) && canConnectTo(from)) {
            e->acceptProposedAction();
        }
    }
}

void Connector::dragMoveEvent(QDragMoveEvent* e)
{
    if(e->mimeData()->text() == Connector::MIME) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());
        overlay_->drawTemporaryLine(QLine(from->centerPoint(), centerPoint()));
    }
}

void Connector::dropEvent(QDropEvent* e)
{
    if(e->mimeData()->text() == Connector::MIME) {
        Connector* from = dynamic_cast<Connector*>(e->mimeData()->parent());

        if(from && from != this) {
            if(tryConnect(from)) {
                overlay_->repaint();
            }
        }
    }
}

void Connector::mousePressEvent(QMouseEvent* e)
{
    if(e->button() == Qt::LeftButton) {
        QDrag* drag = new QDrag(this);
        QMimeData* mimeData = new QMimeData;
        mimeData->setText(Connector::MIME);
        mimeData->setParent(this);
        drag->setMimeData(mimeData);

        drag->exec();

        overlay_->deleteTemporaryLine();
    }
}

void Connector::mouseReleaseEvent(QMouseEvent *e)
{
    if(e->button() == Qt::MiddleButton) {
        removeAllConnections();
    }
}

QPoint Connector::topLeft()
{
    if(box == NULL) {
        findParents();
    }

    return box->geometry().topLeft() + pos();
}

QPoint Connector::centerPoint()
{
    return topLeft() + 0.5 * (geometry().bottomRight() - geometry().topLeft());
}

void Connector::paintEvent(QPaintEvent* e)
{
    setAutoExclusive(false);
    setChecked(isConnected());

    QRadioButton::paintEvent(e);
}
