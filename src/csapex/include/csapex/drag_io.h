#ifndef DRAG_IO_H
#define DRAG_IO_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <QDragEnterEvent>
#include <vector>

namespace csapex{

class DragIO : public Singleton<DragIO>
{
    friend class Singleton<DragIO>;

public:
    struct HandlerEnter {
        typedef boost::shared_ptr<HandlerEnter> Ptr;
        virtual ~HandlerEnter() {}
        virtual bool handle(QWidget *src, Overlay* overlay, QDragEnterEvent* e) = 0;
    };
    struct HandlerMove {
        typedef boost::shared_ptr<HandlerMove> Ptr;
        virtual ~HandlerMove() {}
        virtual bool handle(QWidget *src, Overlay* overlay, QDragMoveEvent* e) = 0;
    };
    struct HandlerDrop {
        typedef boost::shared_ptr<HandlerDrop> Ptr;
        virtual ~HandlerDrop() {}
        virtual bool handle(QWidget *src, Overlay* overlay, QDropEvent* e) = 0;
    };

public:
    void dragEnterEvent(QWidget *src, Overlay* overlay, QDragEnterEvent* e);
    void dragMoveEvent(QWidget *src, Overlay* overlay, QDragMoveEvent* e);
    void dropEvent(QWidget *src, Overlay* overlay, QDropEvent* e);

    static void registerEnterHandler(HandlerEnter::Ptr h);
    static void registerMoveHandler(HandlerMove::Ptr h);
    static void registerDropHandler(HandlerDrop::Ptr h);

private:
    void doRegisterEnterHandler(HandlerEnter::Ptr h);
    void doRegisterMoveHandler(HandlerMove::Ptr h);
    void doRegisterDropHandler(HandlerDrop::Ptr h);

    std::vector<HandlerEnter::Ptr> handler_enter;
    std::vector<HandlerMove::Ptr> handler_move;
    std::vector<HandlerDrop::Ptr> handler_drop;
};

}

#endif // DRAG_IO_H
