/// HEADER
#include "selector_proxy.h"
#include "box.h"
#include "boxed_object.h"

/// PROJECT
#include <designer/box_manager.h>

using namespace vision_evaluator;

SelectorProxy::SelectorProxy(const std::string& type, BoxedObject* prototype, QWidget* parent)
    : QGraphicsView(parent), type_(type), prototype_box_(new vision_evaluator::Box(prototype, type))
{
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    QSize size(80, 80);

    setScene(new QGraphicsScene(QRectF(QPoint(), size)));
    scene()->addPixmap(prototype_box_->makePixmap(type_));
}

SelectorProxy::~SelectorProxy()
{
}

void SelectorProxy::registerProxy(SelectorProxy::Ptr prototype)
{
    BoxManager::instance().register_box_type(prototype);
}

void SelectorProxy::startObjectPositioning(const QPoint& offset)
{
    QDrag* drag = new QDrag(this);
    QMimeData* mimeData = new QMimeData;
    mimeData->setText(Box::MIME);
    mimeData->setParent(this);
    mimeData->setProperty("ox", offset.x());
    mimeData->setProperty("oy", offset.y());
    drag->setMimeData(mimeData);

    vision_evaluator::Box* object(new vision_evaluator::Box(makeContent(), ""));
    object->setObjectName(type_.c_str());
    object->setLabel(type_);
    object->setType(type_);
    object->init(QPoint(0,0));
    object->getContent()->setTypeName(type_);

    QPixmap pm = QPixmap::grabWidget(object);

    delete object;

    drag->setPixmap(pm);
    drag->setHotSpot(-offset);
    drag->exec();
}

void SelectorProxy::mousePressEvent(QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton) {
        startObjectPositioning();
    }
}

vision_evaluator::Box* SelectorProxy::spawnObject(QWidget* parent, const QPoint& pos, const std::string& type, const std::string& uuid)
{
    vision_evaluator::Box* object(new vision_evaluator::Box(makeContent(), uuid, parent));
    object->setObjectName(uuid.c_str());
    object->setType(type);
    object->init(pos);
    object->show();
    object->getContent()->setTypeName(type_);

    return object;
}

std::string SelectorProxy::getType()
{
    return type_;
}

std::string SelectorProxy::getCategory()
{
    BoxedObject* b = makeContent();
    std::string cat = b->getCategory();
    delete b;

    return cat;

}

QIcon SelectorProxy::getIcon()
{
    return prototype_box_->getContent()->getIcon();
}
