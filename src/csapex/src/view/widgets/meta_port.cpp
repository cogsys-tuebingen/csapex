/// HEADER
#include <csapex/view/widgets/meta_port.h>

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/msg/any_message.h>
#include <csapex/csapex_mime.h>

/// SYSTEM
#include <QDragEnterEvent>
#include <QMimeData>
#include <iostream>
#include <QMenu>
#include <QInputDialog>
#include <QApplication>

using namespace csapex;

MetaPort::MetaPort(ConnectorType port_type, const AUUID& target, QWidget *parent)
    : Port(parent), port_type_(port_type), target(target)
{
    setContextMenuPolicy(Qt::CustomContextMenu);

    QObject::connect(this, &MetaPort::customContextMenuRequested, this, &MetaPort::showContextMenu);
}

void MetaPort::showContextMenu(const QPoint& pos)
{
    QMenu menu("Port");

    QString type = QString::fromStdString(port_type::name(port_type_));
    QAction add_port(QString("Add new ") + type, &menu);
    QObject::connect(&add_port, &QAction::triggered, this, &MetaPort::triggerCreatePort);
    menu.addAction(&add_port);

    menu.exec(QCursor::pos());
}

void MetaPort::triggerCreatePort()
{
    bool ok = false;
    QString label = QInputDialog::getText(QApplication::activeWindow(), "Label", "Enter a new label",
                                         QLineEdit::Normal, "", &ok);
    if(!ok) {
        return;
    }

    bool optional = true;

    TokenDataPtr type(new connection_types::AnyMessage);
    Q_EMIT createPortRequest(CreateConnectorRequest(target, port_type_, type, label.toStdString(), optional));
}

void MetaPort::dragEnterEvent(QDragEnterEvent* e)
{
    if(e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_create))) {
        e->acceptProposedAction();
    } else if(e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_move))) {
        e->acceptProposedAction();
    }
}

void MetaPort::dragMoveEvent(QDragMoveEvent* e)
{
    if(e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_create)) ||
            e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_move))) {
        e->acceptProposedAction();
    }
}

void MetaPort::dropEvent(QDropEvent* e)
{
    if(e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_create))) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());
        if(from) {
            auto type = from->getType();

            bool ok = false;
            QString label = QInputDialog::getText(QApplication::activeWindow(), "Label", "Enter a new label",
                                                 QLineEdit::Normal, QString::fromStdString(from->getLabel()), &ok);
            if(!ok) {
                return;
            }

            Q_EMIT createPortAndConnectRequest(CreateConnectorRequest(target, port_type_, type, label.toStdString()), from);
        }

    } else if(e->mimeData()->hasFormat(QString::fromStdString(csapex::mime::connection_move))) {
        Connectable* from = static_cast<Connectable*>(e->mimeData()->property("connectable").value<void*>());
        if(from) {
            auto type = from->getType();

            bool ok = false;
            QString label = QInputDialog::getText(QApplication::activeWindow(), "Label", "Enter a new label",
                                                 QLineEdit::Normal, QString::fromStdString(from->getLabel()), &ok);
            if(!ok) {
                return;
            }


            Q_EMIT createPortAndMoveRequest(CreateConnectorRequest(target, port_type_, type, label.toStdString()), from);
        }

    }
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_meta_port.cpp"

