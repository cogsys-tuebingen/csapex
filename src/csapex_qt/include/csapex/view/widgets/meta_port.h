#ifndef META_PORT_H
#define META_PORT_H

/// COMPONENT
#include <csapex_qt/export.h>

/// PROJECT
#include <csapex/view/widgets/port.h>
#include <csapex/model/connector_description.h>

namespace csapex
{
class CSAPEX_QT_EXPORT MetaPort : public Port
{
    Q_OBJECT

    Q_PROPERTY(QString class READ cssClass)

public:
    MetaPort(ConnectorType port_type, const AUUID& target, QWidget* parent = nullptr);

    QString cssClass()
    {
        return QString("MetaPort");
    }

    void showContextMenu(const QPoint& pt);

    void dragEnterEvent(QDragEnterEvent* e);
    void dragMoveEvent(QDragMoveEvent* e);
    void dropEvent(QDropEvent* e);

Q_SIGNALS:
    void createPortRequest(ConnectorDescription request);
    void createPortAndConnectRequest(ConnectorDescription request, ConnectorPtr);
    void createPortAndMoveRequest(ConnectorDescription request, ConnectorPtr);

protected:
    void createToolTip() override;

private Q_SLOTS:
    void triggerCreatePort();

private:
    ConnectorType port_type_;
    AUUID target;
};

}  // namespace csapex

#endif  // META_PORT_H
