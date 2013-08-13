#ifndef QINTERACTIVE_SCENE_H
#define QINTERACTIVE_SCENE_H
/// SYSTEM
#include <QGraphicsScene>

class QInteractiveScene : public QGraphicsScene
{
public:
    enum Mode{NONE, ADD, MOVE, SELECT, REMOVE};

    QInteractiveScene(QGraphicsView *parent);


    void clearAll();

    void setBackground(QGraphicsItem *item);
    void removeBackground();

    QList<QGraphicsItem *> interactive() const;
    void addInteractive(QGraphicsItem *item);
    void removeInteractive(QGraphicsItem *item);

    void addOverlay(QGraphicsItem *item);
    /// METHOD TO HIDE OVERLAY
    /// METHOD TO HIDE BACKGROUND LAYER
    void clearOverlay();

    bool collision(QGraphicsItem *item);

    bool onBackground(QGraphicsItem *item);

    void selectAll();

    void deselectAll();

    void setMode(const Mode mode);

    Mode getMode();

protected:
    typedef QList<QGraphicsItem*> Layer;
    Mode                 mode_;
    Layer                interactive_;
    Layer                overlay_;
    QGraphicsItem* background_;

};
#endif // QINTERACTIVE_SCENE_H
