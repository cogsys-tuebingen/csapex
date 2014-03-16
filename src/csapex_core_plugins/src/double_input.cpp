/// HEADER
#include "double_input.h"

/// PROJECT

#include <csapex/model/connector_out.h>
#include <csapex_core_plugins/double_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QPushButton>
#include <QBoxLayout>

CSAPEX_REGISTER_CLASS(csapex::DoubleInput, csapex::Node)

using namespace csapex;

DoubleInput::DoubleInput()
    : connector_(NULL)
{
    addTag(Tag::get("Input"));
    addTag(Tag::get("General"));
    setIcon(QIcon(":/pencil.png"));
}

void DoubleInput::process()
{

}

void DoubleInput::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        connector_ = addOutput<connection_types::DoubleMessage>("Double");

        sbox_ = new QDoubleSpinBox;
        QPushButton* send = new QPushButton("ok");

        QHBoxLayout* sub = new QHBoxLayout;

        sub->addWidget(sbox_);
        sub->addWidget(send);

        layout->addLayout(sub);

        connect(sbox_, SIGNAL(valueChanged(double)), this, SLOT(setValue(double)));
        connect(send, SIGNAL(clicked()), this, SLOT(publish()));
    }
}

void DoubleInput::setValue(double value)
{
    state.message = value;
}

void DoubleInput::publish()
{
    connection_types::DoubleMessage::Ptr msg(new connection_types::DoubleMessage);
    msg->value = state.message;
    connector_->publish(msg);
}

void DoubleInput::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "double" << YAML::Value << message;
}

void DoubleInput::State::readYaml(const YAML::Node& node) {
    node["double"] >> message;
}

Memento::Ptr DoubleInput::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void DoubleInput::setState(Memento::Ptr memento)
{
    boost::shared_ptr<DoubleInput::State> m = boost::dynamic_pointer_cast<DoubleInput::State> (memento);
    assert(m.get());

    state = *m;

    sbox_->setValue(state.message);
}

