#ifndef DRAG_IO_H
#define DRAG_IO_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QDragEnterEvent>
#include <vector>
#include <boost/type_traits.hpp>

namespace csapex{

class DragIO
{
public:
    struct HandlerEnter {
        typedef boost::shared_ptr<HandlerEnter> Ptr;
        virtual ~HandlerEnter() {}
        virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, Overlay* overlay, QDragEnterEvent* e) = 0;
    };
    struct HandlerMove {
        typedef boost::shared_ptr<HandlerMove> Ptr;
        virtual ~HandlerMove() {}
        virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, Overlay* overlay, QDragMoveEvent* e) = 0;
    };
    struct HandlerDrop {
        typedef boost::shared_ptr<HandlerDrop> Ptr;
        virtual ~HandlerDrop() {}
        virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, Overlay* overlay, QDropEvent* e) = 0;
    };

public:
    DragIO(CommandDispatcher* dispatcher);

    void dragEnterEvent(QWidget *src, Overlay* overlay, QDragEnterEvent* e);
    void dragMoveEvent(QWidget *src, Overlay* overlay, QDragMoveEvent* e);
    void dropEvent(QWidget *src, Overlay* overlay, QDropEvent* e);

public:
    static bool lock;
    static int grid_size;

public:
    template <typename H>
    void registerHandler() {
        boost::shared_ptr<H> handler(new H);
        if(boost::is_base_of<HandlerEnter,H>::value)
            registerEnterHandler(boost::static_pointer_cast<HandlerEnter>(handler));
        if(boost::is_base_of<HandlerMove,H>::value)
            registerMoveHandler(boost::static_pointer_cast<HandlerMove>(handler));
        if(boost::is_base_of<HandlerDrop,H>::value)
            registerDropHandler(boost::static_pointer_cast<HandlerDrop>(handler));
    }

private:
    void registerEnterHandler(HandlerEnter::Ptr h);
    void registerMoveHandler(HandlerMove::Ptr h);
    void registerDropHandler(HandlerDrop::Ptr h);

    std::vector<HandlerEnter::Ptr> handler_enter;
    std::vector<HandlerMove::Ptr> handler_move;
    std::vector<HandlerDrop::Ptr> handler_drop;

    CommandDispatcher* dispatcher_;
};

}

#endif // DRAG_IO_H
