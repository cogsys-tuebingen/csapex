#ifndef QINTERACTIVE_SCENE_H
#define QINTERACTIVE_SCENE_H
/// SYSTEM
#include <QGraphicsScene>

class QInteractiveScene : public QGraphicsScene
{
public:
    enum Mode{NONE, ADD, MOVE, SELECT, REMOVE};

    QInteractiveScene(QGraphicsView *parent);

    bool collision(QGraphicsItem *item);

    bool onBackground(QGraphicsItem *item);

    void setBackgroudPixmap(QGraphicsPixmapItem *bg);

    void selectAll();

    void deselectAll();

    void clear();

    void setMode(const Mode mode);

    Mode getMode();

protected:
    Mode                 mode_;
    QGraphicsPixmapItem *background;




};
#endif // QINTERACTIVE_SCENE_H
