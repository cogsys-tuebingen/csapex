/// HEADER
#include "int_input.h"

/// PROJECT

#include <csapex/model/connector_out.h>
#include <csapex_core_plugins/int_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QPushButton>
#include <QBoxLayout>

CSAPEX_REGISTER_CLASS(csapex::IntInput, csapex::Node)

using namespace csapex;

IntInput::IntInput()
    : connector_(NULL)
{
    addTag(Tag::get("Input"));
    addTag(Tag::get("General"));
    setIcon(QIcon(":/pencil.png"));
}

void IntInput::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        connector_ = addOutput<connection_types::IntMessage>("Integer");

        sbox_ = new QSpinBox;
        QPushButton* send = new QPushButton("ok");

        QHBoxLayout* sub = new QHBoxLayout;

        sub->addWidget(sbox_);
        sub->addWidget(send);

        layout->addLayout(sub);

        connect(sbox_, SIGNAL(valueChanged(int)), this, SLOT(setValue(int)));
        connect(send, SIGNAL(clicked()), this, SLOT(publish()));
    }
}

void IntInput::setValue(int value)
{
    state.message = value;
}

void IntInput::publish()
{
    connection_types::IntMessage::Ptr msg(new connection_types::IntMessage);
    msg->value = state.message;
    connector_->publish(msg);
}

void IntInput::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "int" << YAML::Value << message;
}

void IntInput::State::readYaml(const YAML::Node& node) {
    node["int"] >> message;
}

Memento::Ptr IntInput::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void IntInput::setState(Memento::Ptr memento)
{
    boost::shared_ptr<IntInput::State> m = boost::dynamic_pointer_cast<IntInput::State> (memento);
    assert(m.get());

    state = *m;

    sbox_->setValue(state.message);
}

