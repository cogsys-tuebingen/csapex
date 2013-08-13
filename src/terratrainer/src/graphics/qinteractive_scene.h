#ifndef QINTERACTIVE_SCENE_H
#define QINTERACTIVE_SCENE_H
/// SYSTEM
#include <QGraphicsScene>

class QInteractiveScene : public QGraphicsScene
{
public:
    typedef QList<QGraphicsItem*> Layer;
    enum Mode{NONE, ADD, MOVE, SELECT, REMOVE};

    QInteractiveScene(QGraphicsView *parent);


    void clearAll();

    void setBackground(QGraphicsItem *item);
    void removeBackground();

    QList<QGraphicsItem *> interactive() const;
    void addInteractive(QGraphicsItem *item);
    void setInteractiveVisible(bool visible = true);
    void removeInteractive(QGraphicsItem *item);

    void addOverlay(QGraphicsItem *item);
    void setOverlay(const Layer &layer);
    void setOverlayVisible(bool visible = false);
    void clearOverlay();

    bool collision(QGraphicsItem *item);

    bool onBackground(QGraphicsItem *item);

    void selectAll();

    void deselectAll();

    void setMode(const Mode mode);

    Mode getMode();

protected:
    Mode                 mode_;
    Layer                interactive_;
    Layer                overlay_;
    QGraphicsItem* background_;

};
#endif // QINTERACTIVE_SCENE_H
