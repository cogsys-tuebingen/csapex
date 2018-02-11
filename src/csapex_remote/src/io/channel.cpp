/// HEADER
#include <csapex/io/channel.h>

/// COMPONENT
#include <csapex/io/note.h>

/// SYSTEM
#include <iostream>

using namespace csapex;
using namespace csapex::io;

Channel::Channel(Session &session, const AUUID &name)
    : session_(session), name_(name)
{
    observe(session_.raw_packet_received(name), raw_packet_received);
}

void Channel::handleNote(const io::NoteConstPtr &note)
{
    note_received(note);
}

Session& Channel::getSession()
{
    return session_;
}
