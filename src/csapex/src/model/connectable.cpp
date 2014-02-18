/// HEADER
#include <csapex/model/connectable.h>

/// COMPONENT
#include <csapex/view/box.h>
#include <csapex/view/design_board.h>
#include <csapex/command/dispatcher.h>
#include <csapex/model/boxed_object.h>
#include <csapex/view/port.h>
#include <csapex/core/settings.h>

/// SYSTEM
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <QDragEnterEvent>
#include <QPainter>

using namespace csapex;

const QString Connectable::MIME_CREATE_CONNECTION = "csapex/connectable/create_connection";
const QString Connectable::MIME_MOVE_CONNECTIONS = "csapex/connectable/move_connections";

//bool Connectable::allow_processing = true;

UUID Connectable::makeUUID(const UUID &box_uuid, int type, int sub_id) {
    if(box_uuid.empty()) {
        return UUID::NONE;
    }

    std::stringstream ss;
    ss << box_uuid << UUID::namespace_separator << (type > 0 ? "in" : (type == 0 ? "out" : "~")) << "_" << sub_id;
    return UUID::make(ss.str());
}

Connectable::Connectable(Settings& settings, const UUID& uuid)
    : Unique(uuid), settings_(settings), buttons_down_(0), minimized_(false), processing(false), enabled_(false), async_(false), async_temp_(false),
      blocked_(false)
{
    init();
}

Connectable::Connectable(Settings& settings, Unique* parent, int sub_id, int type)
    : Unique(makeUUID(parent->getUUID(), type, sub_id)), settings_(settings), buttons_down_(0), minimized_(false), processing(false), enabled_(false), async_(false), async_temp_(false),
      blocked_(false)
{
    init();
}

void Connectable::notifyMessageProcessed()
{
    {
        // QMutexLocker lock(&io_mutex);
        can_process_cond.wakeAll();
    }

    Q_EMIT messageProcessed();
}

void Connectable::updateIsProcessing()
{

}

void Connectable::stop()
{
    {
        QMutexLocker lock(&io_mutex);
        processing = false;
    }
    notifyMessageProcessed();
}

void Connectable::setProcessing(bool p)
{
    {
        QMutexLocker lock(&io_mutex);
        //port_->setPortProperty("processing", p);

        assert(processing != p || isAsync());
        processing = p;
    }


    if(!processing) {
        notifyMessageProcessed();
    }
}

bool Connectable::isProcessing() const
{
    QMutexLocker lock(&io_mutex);
    return processing;
}

void Connectable::waitForProcessing()
{
    if(isError()) {
        return;
    }

    {
        QMutexLocker lock(&io_mutex);

        if(processing) {
            while(processing && settings_.isProcessingAllowed()) {
                blocked_ = true;
                port_->setPortProperty("blocked", true);

                can_process_cond.wait(&io_mutex);
            }
        }


        blocked_ = false;
    }

    port_->setPortProperty("blocked", false);
}

void Connectable::setPort(Port *port)
{
    port_ = port;

    if(isEnabled()) {
        enable();
    } else {
        disable();
    }

    port_->setMinimizedSize(minimized_);
}

Port* Connectable::getPort() const
{
    return port_;
}

CommandDispatcher* Connectable::getCommandDispatcher() const
{
    return dispatcher_;
}

void Connectable::setCommandDispatcher(CommandDispatcher *d)
{
    dispatcher_ = d;
}

void Connectable::init()
{
    port_ = NULL;

    setType(ConnectionType::makeDefault());

    setMinimizedSize(minimized_);

    QObject::connect(this, SIGNAL(connectionRemoved()), this, SLOT(updateIsProcessing()));

    count_ = 0;

    disable();
}


Connectable::~Connectable()
{
}

void Connectable::errorEvent(bool error, const std::string& msg, ErrorLevel level)
{
    port_->setError(error, msg, level);
}

bool Connectable::isForwarding() const
{
    return false;
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
        dispatcher_->execute(removeAllConnectionsCmd());
    }
}

void Connectable::disable()
{
    enabled_ = false;
    if(isProcessing()) {
        setProcessing(false);
    }
    Q_EMIT enabled(enabled_);
    if(port_) {
        port_->setProperty("disabled", !enabled_);
    }
}

void Connectable::enable()
{
    enabled_ = true;
    Q_EMIT enabled(enabled_);
    if(port_) {
        port_->setProperty("disabled", !enabled_);
    }
}

bool Connectable::isEnabled() const
{
    return enabled_;
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

    if(validate) {
        type_ = type;
        validateConnections();
        Q_EMIT typeChanged();
    }
}

ConnectionType::ConstPtr Connectable::getType() const
{
    return type_;
}

void Connectable::setMinimizedSize(bool mini)
{
    minimized_ = mini;

    if(port_) {
        port_->setMinimizedSize(mini);
    }
}

bool Connectable::isMinimizedSize() const
{
    return minimized_;
}

void Connectable::setAsync(bool asynch)
{
    async_ = asynch;
    async_temp_ = asynch;
}

bool Connectable::isAsync() const
{
    return async_ || async_temp_;
}

void Connectable::setTempAsync(bool asynch)
{
    async_temp_ = asynch;
}

int Connectable::getCount() const
{
    return count_;
}

bool Connectable::isBlocked() const
{
    return blocked_;
}
void Connectable::setBlocked(bool b)
{
    blocked_ = b;
}
