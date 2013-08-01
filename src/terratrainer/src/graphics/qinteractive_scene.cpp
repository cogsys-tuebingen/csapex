#include "qinteractive_scene.h"
#include "qinteractive_rect.h"
#include <QGraphicsItem>
#include <QGraphicsView>
#include <QGraphicsItem>

QInteractiveScene::QInteractiveScene(QGraphicsView *parent) :
    QGraphicsScene(parent),
    mode_(NONE)
{
}

void QInteractiveScene::clear()
{
    background = NULL;
    QGraphicsScene::clear();
}

void QInteractiveScene::setBackgroudPixmap(QGraphicsPixmapItem *bg)
{
    background = bg;
    addItem(bg);
}

void QInteractiveScene::selectAll()
{
    foreach (QGraphicsItem* ptr, items()) {
        ptr->setSelected(true);
    }

    background->setSelected(false);
}

void QInteractiveScene::deselectAll()
{
    foreach (QGraphicsItem* ptr, selectedItems()) {
            ptr->setSelected(false);
    }
}

bool QInteractiveScene::onBackground(QGraphicsItem *item)
{
    return background->sceneBoundingRect().contains(item->sceneBoundingRect());
}

bool QInteractiveScene::collision(QGraphicsItem *item)
{
    QList<QGraphicsItem*> collides = collidingItems(item);
    if(collides.size() == 1) {
        return collides[0] != background;
    }
    return collides.size() != 0;
}

void QInteractiveScene::setMode(const Mode mode)
{
    mode_ = mode;
}

QInteractiveScene::Mode QInteractiveScene::getMode()
{
    return mode_;
}
