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

void SelectorProxy::registerProxy(ProxyConstructor c)
{
    BoxManager::instance().register_box_type(c);
}

void SelectorProxy::mousePressEvent(QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton) {
        QDrag* drag = new QDrag(this);
        QMimeData* mimeData = new QMimeData;
        mimeData->setText(Box::MIME);
        mimeData->setParent(this);
        drag->setMimeData(mimeData);
        drag->setPixmap(prototype_box_->makePixmap(type_));
        drag->exec();
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
