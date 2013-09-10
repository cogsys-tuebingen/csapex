/// HEADER
#include <csapex/selector_proxy.h>

/// COMPONENT
#include <csapex/box.h>
#include <csapex/box_group.h>
#include <csapex/boxed_object.h>
#include <csapex/box_manager.h>

using namespace csapex;

const SelectorProxy::Ptr SelectorProxy::NullPtr;

SelectorProxy::SelectorProxy(const std::string& type, const std::string& description, boost::shared_ptr<BoxedObject> prototype)
    : type_(type), descr_(description)
{
    icon = prototype->getIcon();
    cat = prototype->getTags();
}

SelectorProxy::~SelectorProxy()
{
}

void SelectorProxy::registerProxy(SelectorProxy::Ptr prototype)
{
    BoxManager::instance().register_box_type(prototype);
}

void SelectorProxy::startObjectPositioning(QWidget* parent, const QPoint& offset, const std::string& template_)
{
    QDrag* drag = new QDrag(parent);
    QMimeData* mimeData = new QMimeData;

    QString type(type_.c_str());
    mimeData->setData(Box::MIME, type_.c_str());
    mimeData->setProperty("ox", offset.x());
    mimeData->setProperty("oy", offset.y());
    drag->setMimeData(mimeData);

    csapex::Box* object;

    if(type_ == "::meta") {
        object = new csapex::BoxGroup(makeContent(), "");
        mimeData->setData(SubGraphTemplate::MIME, template_.c_str());
    } else {
        object = new csapex::Box(makeContent(), "");
    }

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
//        startObjectPositioning();
    }
}

Box::Ptr SelectorProxy::create(const QPoint& pos, const std::string& type, const std::string& uuid)
{
    csapex::Box::Ptr object;

    if(type_ == "::meta") {
        object.reset(new csapex::BoxGroup(makeContent(), uuid));
    } else {
        object.reset(new csapex::Box(makeContent(), uuid));
    }

    object->setObjectName(uuid.c_str());
    object->setType(type);
    object->init(pos);
    object->getContent()->setTypeName(type_);

    return object;
}

std::string SelectorProxy::getType()
{
    return type_;
}

std::vector<Tag> SelectorProxy::getTags()
{
    return cat;

}

QIcon SelectorProxy::getIcon()
{
    return icon;
}

std::string SelectorProxy::getDescription()
{
    return descr_;
}
