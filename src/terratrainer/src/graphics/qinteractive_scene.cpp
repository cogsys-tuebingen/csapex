#include "qinteractive_scene.h"
#include "qinteractive_rect.h"
#include <QGraphicsItem>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QGraphicsItemGroup>

QInteractiveScene::QInteractiveScene(QGraphicsView *parent) :
    QGraphicsScene(parent),
    mode_(NONE)
{
    clearAll();
}

void QInteractiveScene::clearAll()
{
    overlay_.clear();
    interactive_.clear();
    clear();
}

void QInteractiveScene::setBackground(QGraphicsItem *item)
{
    background_ = item;
    addItem(item);
}

void QInteractiveScene::removeBackground()
{
    removeItem(background_);
    background_ = NULL;
}

QList<QGraphicsItem*> QInteractiveScene::interactive() const
{
    return interactive_;
}

void QInteractiveScene::addInteractive(QGraphicsItem *item)
{

    if(onBackground(item)) {
        interactive_.push_back(item);
        addItem(item);
    }
}

void QInteractiveScene::setInteractiveVisible(bool visible)
{
    for(Layer::Iterator it = interactive_.begin() ; it != interactive_.end() ; it++)
        (*it)->setVisible(visible);
}

void QInteractiveScene::removeInteractive(QGraphicsItem *item)
{
    Layer::Iterator pos = std::find(interactive_.begin(), interactive_.end(), item);
    if(pos != interactive_.end()) {
        removeItem(item);
        interactive_.erase(pos);
    }
}

void QInteractiveScene::addOverlay(QGraphicsItem *item)
{
    for(Layer::Iterator it = interactive_.begin() ; it != interactive_.end() ; it++)
        (*it)->setVisible(false);

    item->setVisible(false);
    overlay_.push_back(item);
    addItem(item);
}

void QInteractiveScene::setOverlay(const Layer &layer)
{
    overlay_ = layer;
    for(Layer::ConstIterator it = layer.begin() ; it != layer.end() ; it++)
        addItem(*it);
}

void QInteractiveScene::setOverlayVisible(bool visible)
{
    for(Layer::Iterator it = overlay_.begin() ; it != overlay_.end() ; it++)
        (*it)->setVisible(visible);
}

void QInteractiveScene::clearOverlay()
{
    for(Layer::Iterator it = interactive_.begin() ; it != interactive_.end() ; it++)
        (*it)->setVisible(true);
    for(Layer::Iterator it = overlay_.begin() ; it != overlay_.end() ; it++)
        removeItem(*it);
    overlay_.clear();
}

void QInteractiveScene::selectAll()
{
    for(Layer::Iterator it = interactive_.begin() ; it != interactive_.end() ; it++)
        (*it)->setSelected(true);
}

void QInteractiveScene::deselectAll()
{
    for(Layer::Iterator it = interactive_.begin() ; it != interactive_.end() ; it++)
        (*it)->setSelected(false);
}

bool QInteractiveScene::onBackground(QGraphicsItem *item)
{
    int test = this->items().size();
    return background_->sceneBoundingRect().contains(item->sceneBoundingRect());
}

bool QInteractiveScene::collision(QGraphicsItem *item)
{
    QList<QGraphicsItem*> collides = collidingItems(item);

    if(collides.size() == 1) {
        return background_ != collides[0];
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
