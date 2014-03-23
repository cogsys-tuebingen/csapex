/// HEADER
#include "text_display.h"

/// PROJECT
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TextDisplay, csapex::Node)

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
        setSynchronizedInputs(true);

        //connector_ = addInput<connection_types::DirectMessage<std::string> >("Text");
        connector_ = addInput<connection_types::AnyMessage>("Anything", false, true);
        txt_ = new QLabel("<i>no input yet</i>");
        layout->addWidget(txt_);
    }
}

void TextDisplay::process()
{
    connection_types::Message::Ptr msg = connector_->getMessage<connection_types::Message>();

    std::stringstream ss;
    msg->write(ss);

    txt_->setText(ss.str().c_str());
}
