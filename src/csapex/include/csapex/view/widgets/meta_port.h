#ifndef META_PORT_H
#define META_PORT_H

/// PROJECT
#include <csapex/view/widgets/port.h>

namespace csapex
{

class MetaPort : public Port
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    MetaPort(QWidget *parent = nullptr);

    QString cssClass() {
        return QString("MetaPort");
    }

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

Q_SIGNALS:
    void createPortAndConnectRequest(Connectable*, ConnectionTypeConstPtr, std::string, bool);
    void createPortAndMoveRequest(Connectable*, ConnectionTypeConstPtr, std::string, bool);
};

}

#endif // META_PORT_H
