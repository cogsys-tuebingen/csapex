/// HEADER
#include "box.h"

/// PROJECT
#include "ui_box.h"

/// SYSTEM
#include <QDragMoveEvent>
#include <QMenu>
#include <iostream>

using namespace vision_evaluator;

Box::Box(QWidget *parent)
    : QWidget(parent), DragTracker(this), ui(new Ui::Box), input(new ConnectorIn(this)), output(new ConnectorOut(this))
{
    ui->setupUi(this);

    ui->input_layout->addWidget(input);
    ui->output_layout->addWidget(output);

    setObjectName("Box");

    setContextMenuPolicy(Qt::CustomContextMenu);

    connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showContextMenu(const QPoint&)));
}

Box::~Box()
{

}

QPixmap Box::makePixmap()
{
    int border = 3;

    QImage img(QSize(80, 80), QImage::Format_ARGB32);
    QPainter painter(&img);
    painter.fillRect(QRect(QPoint(0,0), img.size()), Qt::white);
    painter.setPen(QPen(Qt::red, border));
    painter.drawRect(QRect(QPoint(border/2,border/2), img.size() - QSize(border,border)));
    painter.drawText(QPoint(20, 40), QString("void"));
    return QPixmap::fromImage(img);
}

void Box::setOverlay(Overlay *o)
{
    DragTracker::setOverlay(o);

    input->setOverlay(o);
    output->setOverlay(o);
}

void Box::showContextMenu(const QPoint &pos)
{
    QPoint globalPos = mapToGlobal(pos);

    QString remove_txt("delete");

    QMenu menu;
    menu.addAction(remove_txt);

    QAction* selectedItem = menu.exec(globalPos);

    if (selectedItem) {
        if(selectedItem->text() == remove_txt) {
            deleteLater();
        }
    }
}

void Box::mouseReleaseEvent(QMouseEvent *e)
{
    DragTracker::mouseReleaseEvent(e);

    e->ignore();
}


void Box::mouseDelta(const QPoint& delta)
{
    move(start_pos + delta);
    overlay_->repaint();
}

QPoint Box::getPos() const
{
    return pos();
}
