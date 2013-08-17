/// HEADER
#include <csapex/box_group.h>

/// COMPONENT
#include "ui_box.h"

/// SYSTEM
#include <QDragEnterEvent>

using namespace csapex;

const QString BoxGroup::MIME = "csapex/boxmeta";

BoxGroup::BoxGroup(BoxedObject *content, const std::string &uuid, QWidget *parent)
    : Box(content, uuid, parent)
{
    setAcceptDrops(true);
}

void BoxGroup::dragEnterEvent(QDragEnterEvent *e)
{
    if(e->mimeData()->hasFormat(Box::MIME_MOVE)) {
        if(e->source() == this) {
            return;
        }

        e->accept();
        ui->boxframe->setProperty("highlight", true);
        refreshStylesheet();
    }
}

void BoxGroup::dragLeaveEvent(QDragLeaveEvent *)
{
    ui->boxframe->setProperty("highlight", false);
    refreshStylesheet();
}

void BoxGroup::dropEvent(QDropEvent *e)
{
    ui->boxframe->setProperty("highlight", false);
    refreshStylesheet();

    e->setDropAction(Qt::IgnoreAction);

    Q_EMIT moveSelectionToBox(this);
}


bool BoxGroup::hasSubGraph()
{
    return true;
}

Graph* BoxGroup::getSubGraph()
{
    return &sub_graph;
}
