/// HEADER
#include "panel.h"

#include "image_combiner_panel.h"

using namespace vision_evaluator;

Panel::Panel(QWidget* parent)
    : QWidget(parent), scene(new QGraphicsScene)
{
    worker = new QThread();
    worker->start();
}

Panel::~Panel()
{
}

void Panel::display_request(const QSharedPointer<QImage> image)
{
    Q_EMIT display_request_gui(image);
}


void Panel::display(const QSharedPointer<QImage> qimg)
{
    QPixmap pm = QPixmap::fromImage(*qimg);
    scene->clear();
    scene->addPixmap(pm);

    view->fitInView(scene->itemsBoundingRect() ,Qt::KeepAspectRatio);
}
