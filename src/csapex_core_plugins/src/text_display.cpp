/// HEADER
#include "text_display.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex_core_plugins/string_message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::TextDisplay, csapex::BoxedObject)

using namespace csapex;

TextDisplay::TextDisplay()
    : connector_(NULL)
{
    addTag(Tag::get("Output"));
    addTag(Tag::get("General"));
    setIcon(QIcon(":/pencil.png"));
}

void TextDisplay::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        box_->setSynchronizedInputs(true);

        connector_ = box_->addInput<connection_types::StringMessage>("Text");
        txt_ = new QLabel("<i>no input yet</i>");
        layout->addWidget(txt_);
    }
}

void TextDisplay::allConnectorsArrived()
{
    connection_types::StringMessage::Ptr msg = connector_->getMessage<connection_types::StringMessage>();

    txt_->setText(msg->value.c_str());
}
