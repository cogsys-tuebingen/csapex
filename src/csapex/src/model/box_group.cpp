/// HEADER
#include <csapex/model/box_group.h>

/// COMPONENT
#include <csapex/model/boxed_object.h>
#include <csapex/manager/template_manager.h>
#include "ui_box.h"

/// SYSTEM
#include <QDragEnterEvent>

using namespace csapex;

const QString BoxGroup::MIME = "csapex/model/boxmeta";

BoxGroup::BoxGroup(const std::string &uuid, QWidget *parent)
    : Box(BoxedObject::Ptr(new NullBoxedObject), uuid, parent), sub_graph(new Graph)
{
    icon_ = new QLabel();
    QIcon img(":/group.png");
    icon_->setPixmap(img.pixmap(QSize(16,16)));
    ui->additional_layout->addWidget(icon_);
}

bool BoxGroup::hasSubGraph()
{
    return true;
}

Graph::Ptr BoxGroup::getSubGraph()
{
    return sub_graph;
}

void BoxGroup::init(const QPoint &pos)
{
    Box::init(pos);

    Template::Ptr templ = TemplateManager::instance().get(BoxManager::getTemplateName(getType()));
    command::Meta::Ptr meta(new command::Meta);
    templ->createCommands(meta.get(), UUID());

    Command::doExecute(meta);
}
