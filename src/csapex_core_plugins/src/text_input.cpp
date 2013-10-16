/// HEADER
#include "text_input.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex_core_plugins/string_message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QLineEdit>
#include <QPushButton>
#include <QBoxLayout>

PLUGINLIB_EXPORT_CLASS(csapex::TextInput, csapex::BoxedObject)

using namespace csapex;

TextInput::TextInput()
    : connector_(NULL)
{
    addTag(Tag::get("Input"));
    addTag(Tag::get("General"));
    setIcon(QIcon(":/pencil.png"));
}

void TextInput::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        connector_ = box_->addOutput<connection_types::StringMessage>("Text");

        txt_ = new QLineEdit;
        QPushButton* send = new QPushButton("ok");

        QHBoxLayout* sub = new QHBoxLayout;

        sub->addWidget(txt_);
        sub->addWidget(send);

        layout->addLayout(sub);

        connect(txt_, SIGNAL(textChanged(QString)), this, SLOT(setText(QString)));
        connect(send, SIGNAL(clicked()), this, SLOT(publish()));
    }
}

void TextInput::setText(QString text)
{
    state.message = text.toStdString();
}

void TextInput::publish()
{
    connection_types::StringMessage::Ptr msg(new connection_types::StringMessage);
    msg->value = state.message;
    connector_->publish(msg);
}

void TextInput::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "message" << YAML::Value << message;
}

void TextInput::State::readYaml(const YAML::Node& node) {
    node["message"] >> message;
}

Memento::Ptr TextInput::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void TextInput::setState(Memento::Ptr memento)
{
    boost::shared_ptr<TextInput::State> m = boost::dynamic_pointer_cast<TextInput::State> (memento);
    assert(m.get());

    state = *m;

    txt_->setText(state.message.c_str());
}

