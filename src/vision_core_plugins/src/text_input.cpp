/// HEADER
#include "text_input.h"

/// PROJECT
#include <designer/box.h>
#include <designer/connector_out.h>
#include <vision_evaluator/messages_default.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QLineEdit>
#include <QPushButton>
#include <QBoxLayout>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::TextInput, vision_evaluator::BoxedObject)

using namespace vision_evaluator;

TextInput::TextInput()
    : connector_(NULL)
{
    setCategory("Value Input");
    setIcon(QIcon(":/pencil.png"));
}

void TextInput::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        connector_ = new ConnectorOut(box_, 0);
        connector_->setLabel("Text");
        connector_->setType(connection_types::StringMessage::make());

        box_->addOutput(connector_);

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

void TextInput::messageArrived(ConnectorIn *source)
{
    // NO INPUT
}

void TextInput::setText(QString text)
{
    state.message = text.toUtf8().constData();
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
    boost::shared_ptr<TextInput::State> memento(new TextInput::State);
    *memento = state;

    return memento;
}

void TextInput::setState(Memento::Ptr memento)
{
    boost::shared_ptr<TextInput::State> m = boost::dynamic_pointer_cast<TextInput::State> (memento);
    assert(m.get());

    state = *m;

    txt_->setText(state.message.c_str());
}

