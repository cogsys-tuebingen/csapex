/// HEADER
#include <csapex/model/box_group.h>

/// COMPONENT
#include <csapex/model/boxed_object.h>
#include <csapex/manager/template_manager.h>
#include <csapex/core/graphio.h>
#include <csapex/command/dispatcher.h>
#include <csapex/view/template_dialog.h>
#include "ui_box.h"

/// SYSTEM
#include <QDragEnterEvent>
#include <QMessageBox>

using namespace csapex;

const QString BoxGroup::MIME = "csapex/model/boxmeta";

BoxGroup::BoxGroup(const std::string &type, const std::string &uuid, QWidget *parent)
    : Box(BoxedObject::Ptr(new NullBoxedObject(type)), BoxedObject::Ptr(new NullBoxedObject(type)), uuid, parent)
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

void BoxGroup::fillContextMenu(QMenu *menu, std::map<QAction*, boost::function<void()> >& handler)
{
    Box::fillContextMenu(menu, handler);

    menu->addSeparator();

    QAction* del = new QAction("save as template", menu);
    del->setIcon(QIcon(":/group.png"));
    del->setIconVisibleInMenu(true);
    handler[del] = boost::bind(&BoxGroup::saveAsTemplate, this);
    menu->addAction(del);
}

void BoxGroup::saveAsTemplate()
{
    TemplateDialog diag;
    int r = diag.exec();

    if(r) {
        std::cout << "!SAVE! " << diag.getName() << std::endl;
        Template::Ptr templ = cmd_dispatcher.getGraph()->toTemplate(diag.getName());
        std::string path = TemplateManager::defaultTemplatePath() + diag.getName() + GraphIO::template_extension;

        TemplateManager::instance().save(path, templ);
    }
}
