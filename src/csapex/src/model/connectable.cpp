/// HEADER
#include <csapex/model/connectable.h>

/// SYSTEM
#include <iostream>
#include <sstream>
#include <typeinfo>

using namespace csapex;

const QString Connectable::MIME_CREATE_CONNECTION = "csapex/connectable/create_connection";
const QString Connectable::MIME_MOVE_CONNECTIONS = "csapex/connectable/move_connections";

//bool Connectable::allow_processing = true;

UUID Connectable::makeUUID(const UUID &box_uuid, const std::string& type, int sub_id) {
    if(box_uuid.empty()) {
        return UUID::NONE;
    }

    std::stringstream ss;
    ss << box_uuid << UUID::namespace_separator << type << "_" << sub_id;
    return UUID::make(ss.str());
}


Connectable::Connectable(const UUID& uuid)
    : Unique(uuid),
      buttons_down_(0), count_(0), seq_no_(0), enabled_(false),
      blocked_(false)
{
    init();
}

Connectable::Connectable(Unique* parent, int sub_id, const std::string& type)
    : Unique(makeUUID(parent->getUUID(), type, sub_id)),
      buttons_down_(0), count_(0), seq_no_(0), enabled_(false),
      blocked_(false)
{
    init();
}

void Connectable::notifyMessageProcessed()
{
    Q_EMIT messageProcessed(this);
}

void Connectable::stop()
{
    notifyMessageProcessed();
}

void Connectable::init()
{
    setType(ConnectionType::makeDefault());

    disable();
}


Connectable::~Connectable()
{
    //    UUID::free(getUUID());
}

void Connectable::errorEvent(bool error, const std::string& msg, ErrorLevel level)
{
    Q_EMIT connectableError(error,msg,static_cast<int>(level));
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

void Connectable::disable()
{
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
    if(enabled_) {
        enabled_ = false;
        Q_EMIT enabled(enabled_);
    }
}

void Connectable::enable()
{
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
    if(!enabled_) {
        enabled_ = true;
        Q_EMIT enabled(enabled_);
    }
}

void Connectable::setEnabled(bool enabled)
{
    if(enabled) {
        enable();
    } else {
        disable();
    }
}

bool Connectable::isEnabled() const
{
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
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
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
    return label_;
}

void Connectable::setLabel(const std::string &label)
{
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
    label_ = label;
}

void Connectable::setType(ConnectionType::ConstPtr type)
{
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
    bool validate = type_ != type;

    if(validate) {
        type_ = type;
        validateConnections();
        Q_EMIT typeChanged();
    }
}

ConnectionType::ConstPtr Connectable::getType() const
{
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
    return type_;
}

int Connectable::getCount() const
{
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
    return count_;
}

bool Connectable::isBlocked() const
{
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
    return blocked_;
}
void Connectable::setBlocked(bool b)
{
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
    blocked_ = b;
//    Q_EMIT blocked(b);
}

int Connectable::sequenceNumber() const
{
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
    return seq_no_;
}

void Connectable::setSequenceNumber(int seq_no)
{
    std::lock_guard<std::recursive_mutex> lock(sync_mutex);
    seq_no_ = seq_no;
}
/// MOC
#include "../../include/csapex/model/moc_connectable.cpp"
