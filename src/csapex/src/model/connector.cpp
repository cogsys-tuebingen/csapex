/// HEADER
#include <csapex/model/connector.h>

/// COMPONENT
#include <csapex/view/box.h>
#include <csapex/view/design_board.h>
#include <csapex/command/dispatcher.h>
#include <csapex/model/node.h>
#include <csapex/model/boxed_object.h>
#include <csapex/view/port.h>

/// SYSTEM
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <QDragEnterEvent>
#include <QPainter>

using namespace csapex;

const std::string Connectable::namespace_separator = ":|:";

const QString Connectable::MIME_CREATE_CONNECTION = "csapex/connectable/create_connection";
const QString Connectable::MIME_MOVE_CONNECTIONS = "csapex/connectable/move_connections";

std::string Connectable::makeUUID(const std::string& box_uuid, int type, int sub_id) {
    std::stringstream ss;
    ss << box_uuid << namespace_separator << (type > 0 ? "in" : (type == 0 ? "out" : "~")) << "_" << sub_id;
    return ss.str();
}

Connectable::Connectable(Node* parent, const std::string& uuid)
    : node_(parent), buttons_down_(0), uuid_(uuid), minimized_(false)
{
    init();
}

Connectable::Connectable(Node* parent, int sub_id, int type)
    : node_(parent), buttons_down_(0), uuid_(makeUUID(parent->UUID(), type, sub_id)), minimized_(false)
{
    init();
}

void Connectable::setPort(Port *port)
{
    port_ = port;
}

Port* Connectable::getPort() const
{
    return port_;
}

void Connectable::init()
{
    port_ = NULL;

    setParent(node_->getBox());

    setType(ConnectionType::makeDefault());

    setMinimizedSize(minimized_);

    count_ = 0;
}


Connectable::~Connectable()
{
}

void Connectable::errorEvent(bool error, const std::string& msg, ErrorLevel level)
{
    node_->setError(error, msg, level);
}

void Connectable::errorChanged(bool error)
{
    setProperty("error", error);
    //refreshStylesheet();
}

bool Connectable::isForwarding() const
{
    return false;
}

std::string Connectable::UUID() const
{
    return uuid_;
}

bool Connectable::tryConnect(QObject* other_side)
{
    Connectable* c = dynamic_cast<Connectable*>(other_side);
    if(c) {
        return tryConnect(c);
    }
    return false;
}

void Connectable::removeConnection(QObject* other_side)
{
    Connectable* c = dynamic_cast<Connectable*>(other_side);
    if(c) {
        removeConnection(c);
    }
}

void Connectable::validateConnections()
{

}

void Connectable::removeAllConnectionsUndoable()
{
    if(isConnected()) {
        getNode()->getCommandDispatcher()->execute(removeAllConnectionsCmd());
    }
}

void Connectable::disable()
{
    port_->setEnabled(false);
    Q_EMIT disabled(this);
    port_->setProperty("disabled", true);
    //refreshStylesheet();
}

void Connectable::enable()
{
    port_->setEnabled(true);
    Q_EMIT enabled(this);
    port_->setProperty("disabled", false);
    //refreshStylesheet();
}

bool Connectable::canConnectTo(Connectable* other_side, bool) const
{
    if(other_side == this) {
        return false;
    }

    bool in_out = (canOutput() && other_side->canInput()) || (canInput() && other_side->canOutput());
    bool compability = getType()->canConnectTo(other_side->getType().get());

    return in_out && compability;
}


bool Connectable::shouldCreate(bool left, bool)
{
    bool full_input = isInput() && isConnected();
    return left && !full_input;
}

bool Connectable::shouldMove(bool left, bool right)
{
    bool full_input = isInput() && isConnected();
    return (right && isConnected()) || (left && full_input);
}

std::string Connectable::getLabel() const
{
    return label_;
}

void Connectable::setLabel(const std::string &label)
{
    label_ = label;
}

void Connectable::setType(ConnectionType::ConstPtr type)
{
    bool validate = type_ != type;
    type_ = type;

    if(validate) {
        validateConnections();
    }
}

ConnectionType::ConstPtr Connectable::getType() const
{
    return type_;
}

void Connectable::setMinimizedSize(bool mini)
{
    minimized_ = mini;

//    if(mini) {
//        setFixedSize(8,8);
//    } else {
//        setFixedSize(16,16);
//    }
}

bool Connectable::isMinimizedSize() const
{
    return minimized_;
}

Node* Connectable::getNode() const
{
    QMutexLocker lock(&mutex);
    return node_;
}

int Connectable::getCount() const
{
    return count_;
}

