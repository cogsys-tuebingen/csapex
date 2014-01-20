/// HEADER
#include <csapex/model/group.h>

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

const QString Group::MIME = "csapex/model/boxmeta";

Group::Group(const std::string &/*type*/, const UUID& /*uuid*/, QWidget *parent)
    : Box(BoxedObject::Ptr((BoxedObject*) NULL), BoxedObject::Ptr((BoxedObject*) NULL), parent)
{
    icon_ = new QLabel();
    QIcon img(":/group.png");
    icon_->setPixmap(img.pixmap(QSize(16,16)));
    ui->additional_layout->addWidget(icon_);
}

bool Group::eventFilter(QObject * o, QEvent * e)
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

bool Group::hasSubGraph()
{
    return true;
}

Graph::Ptr Group::getSubGraph()
{
    return cmd_dispatcher.getGraph();
}

//void Group::init()
//{
//    Box::init();

//    Template::Ptr templ = TemplateManager::instance().get(BoxManager::getTemplateName(getType()));
//    assert(templ);
//    command::Meta::Ptr meta(new command::Meta);
//    templ->createCommands(meta.get(), UUID());

//    node_->getCommandDispatcher()->executeNotUndoable(meta);
//}

void Group::fillContextMenu(QMenu *menu, std::map<QAction*, boost::function<void()> >& handler)
{
    Box::fillContextMenu(menu, handler);

    menu->addSeparator();

    QAction* del = new QAction("save as template", menu);
    del->setIcon(QIcon(":/group.png"));
    del->setIconVisibleInMenu(true);
    handler[del] = boost::bind(&Group::saveAsTemplate, this);
    menu->addAction(del);
}

void Group::saveAsTemplate()
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
