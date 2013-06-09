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
    : QWidget(parent), ui(new Ui::Box), input(new ConnectorIn(this)), output(new ConnectorOut(this)), down_(false)
{
    ui->setupUi(this);

    ui->input_layout->addWidget(input);
    ui->output_layout->addWidget(output);

    setObjectName(ui->content->title());

    ui->content->installEventFilter(this);

    setContextMenuPolicy(Qt::CustomContextMenu);

    connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showContextMenu(const QPoint&)));
}

Box::~Box()
{

}

bool Box::eventFilter(QObject * o, QEvent * e)
{
    if(o == ui->content) {
        if(e->type() == QEvent::MouseButtonPress) {
            down_ = true;
        } else if(e->type() == QEvent::MouseButtonRelease) {
            down_ = false;
        } else if(e->type() == QEvent::MouseMove) {
            if(down_) {
                e->ignore();
            }
        }
    }

    return false;
}

void Box::paintEvent(QPaintEvent *e)
{
    ui->content->setTitle(objectName());
    resize(sizeHint());
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

QPixmap Box::makePixmap(const std::string& label)
{
    int border = 3;

    QImage img(QSize(80, 80), QImage::Format_ARGB32);
    QPainter painter(&img);
    painter.fillRect(QRect(QPoint(0,0), img.size()), Qt::white);
    painter.setPen(QPen(Qt::red, border));
    painter.drawRect(QRect(QPoint(border/2,border/2), img.size() - QSize(border,border)));
    painter.drawText(QPoint(5, 40), label.c_str());
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
