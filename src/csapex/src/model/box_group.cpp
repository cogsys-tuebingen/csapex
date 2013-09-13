/// HEADER
#include <csapex/model/box_group.h>

/// COMPONENT
#include <csapex/model/boxed_object.h>
#include <csapex/manager/template_manager.h>
#include <csapex/command/dispatcher.h>
#include "ui_box.h"

/// SYSTEM
#include <QDragEnterEvent>
#include <QMessageBox>

using namespace csapex;

const QString BoxGroup::MIME = "csapex/model/boxmeta";

BoxGroup::BoxGroup(const std::string &uuid, QWidget *parent)
    : Box(BoxedObject::Ptr(new NullBoxedObject), uuid, parent)
{
    icon_ = new QLabel();
    QIcon img(":/group.png");
    icon_->setPixmap(img.pixmap(QSize(16,16)));
    ui->additional_layout->addWidget(icon_);
}

bool BoxGroup::eventFilter(QObject * o, QEvent * e)
{
    if (e->type() == QEvent::MouseButtonDblClick) {
        QMouseEvent * me = static_cast <QMouseEvent *> (e);

        if (me->button() == Qt::LeftButton) {
            if(o != ui->label) {
                Q_EMIT open_sub_graph(this);
                return true;
            }
        }
    }

    return Box::eventFilter(o, e);
}

bool BoxGroup::hasSubGraph()
{
    return true;
}

Graph::Ptr BoxGroup::getSubGraph()
{
    return cmd_dispatcher.getGraph();
}

void BoxGroup::init(const QPoint &pos)
{
    Box::init(pos);

    Template::Ptr templ = TemplateManager::instance().get(BoxManager::getTemplateName(getType()));
    assert(templ);
    command::Meta::Ptr meta(new command::Meta);
    templ->createCommands(meta.get(), UUID());

    dispatcher_->executeNotUndoable(meta);
}
