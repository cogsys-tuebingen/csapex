#ifndef META_PORT_H
#define META_PORT_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// PROJECT
#include <csapex/view/widgets/port.h>
#include <csapex/utility/create_connector_request.h>

namespace csapex
{

class CSAPEX_QT_EXPORT MetaPort : public Port
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    MetaPort(ConnectorType port_type, const AUUID &target, QWidget *parent = nullptr);

    QString cssClass() {
        return QString("MetaPort");
    }

    void showContextMenu(const QPoint &pt);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

Q_SIGNALS:
    void createPortRequest(CreateConnectorRequest request);
    void createPortAndConnectRequest(CreateConnectorRequest request, Connectable*);
    void createPortAndMoveRequest(CreateConnectorRequest request, Connectable*);

private Q_SLOTS:
    void triggerCreatePort();

private:
    ConnectorType port_type_;
    AUUID target;
};

}

#endif // META_PORT_H
