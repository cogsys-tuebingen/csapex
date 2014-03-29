/// HEADER
#include "text_display.h"

/// PROJECT

#include <csapex/model/connector_in.h>
#include <csapex_core_plugins/string_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TextDisplay, csapex::Node)

using namespace csapex;

TextDisplay::TextDisplay()
    : connector_(NULL)
{
    addTag(Tag::get("Output"));
    addTag(Tag::get("General"));
}

QIcon TextDisplay::getIcon() const
{
    return QIcon(":/pencil.png");
}

void TextDisplay::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        setSynchronizedInputs(true);

        connector_ = addInput<connection_types::StringMessage>("Text");
        txt_ = new QLabel("<i>no input yet</i>");
        layout->addWidget(txt_);
    }
}

void TextDisplay::process()
{
    connection_types::StringMessage::Ptr msg = connector_->getMessage<connection_types::StringMessage>();

    txt_->setText(msg->value.c_str());
}
