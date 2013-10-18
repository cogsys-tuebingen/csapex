/// HEADER
#include "say_text.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex_core_plugins/string_message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::SayText, csapex::BoxedObject)

using namespace csapex;

SayText::SayText()
    : connector_(NULL)
{
    addTag(Tag::get("Output"));
    addTag(Tag::get("General"));
    setIcon(QIcon(":/pencil.png"));
}

void SayText::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        box_->setSynchronizedInputs(true);

        connector_ = box_->addInput<connection_types::StringMessage>("Text");
    }
}

void SayText::allConnectorsArrived()
{
    connection_types::StringMessage::Ptr msg = connector_->getMessage<connection_types::StringMessage>();

    std::stringstream cmd;
    cmd << "espeak \"" << msg->value << "\" 2> /dev/null 1> /dev/null ";
    system(cmd.str().c_str());
}
