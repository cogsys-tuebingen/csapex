/// HEADER
#include "box.h"

/// PROJECT
#include "ui_box.h"

/// SYSTEM
#include <QDragMoveEvent>
#include <QMenu>
#include <iostream>

using namespace vision_evaluator;

const QString Box::MIME = "vision_evaluator/box";
const QString Box::MIME_MOVE = "vision_evaluator/box/move";

Box::Box(QWidget* parent)
    : QWidget(parent), ui(new Ui::Box), input(new ConnectorIn(this)), output(new ConnectorOut(this))
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

void Box::mousePressEvent(QMouseEvent* e)
{
    if(e->button() == Qt::LeftButton) {
        QDrag* drag = new QDrag(this);
        QMimeData* mimeData = new QMimeData;
        mimeData->setText(Box::MIME_MOVE);
        mimeData->setParent(this);
        mimeData->setUserData(0, new MoveOffset(-e->pos()));
        drag->setMimeData(mimeData);

        drag->exec();
    }
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

void Box::setOverlay(Overlay* o)
{
    overlay_ = o;

    input->setOverlay(o);
    output->setOverlay(o);
}

void Box::showContextMenu(const QPoint& pos)
{
    QPoint globalPos = mapToGlobal(pos);

    QString remove_txt("delete");

    QMenu menu;
    menu.addAction(remove_txt);

    QAction* selectedItem = menu.exec(globalPos);

    if(selectedItem) {
        if(selectedItem->text() == remove_txt) {
            deleteLater();
        }
    }
}
