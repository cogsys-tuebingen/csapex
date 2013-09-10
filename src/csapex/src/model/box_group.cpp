/// HEADER
#include <csapex/model/box_group.h>

/// COMPONENT
#include <csapex/model/boxed_object.h>
#include "ui_box.h"

/// SYSTEM
#include <QDragEnterEvent>

using namespace csapex;

const QString BoxGroup::MIME = "csapex/model/boxmeta";

BoxGroup::BoxGroup(BoxedObject::Ptr content, const std::string &uuid, QWidget *parent)
    : Box(content, uuid, parent), sub_graph(new Graph)
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

void BoxGroup::setTemplateName(const std::string &templ)
{
    Box::setTemplateName(templ);

    icon_->setToolTip((std::string("template: ") + state->template_).c_str());
}
