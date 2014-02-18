/// HEADER
#include "foreach.h"

/// COMPONENT
#include <csapex_core_plugins/vector_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex/view/port.h>
#include <csapex/utility/timer.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Foreach, csapex::Node)

using namespace csapex;
using namespace connection_types;

Foreach::Foreach()
    : in_sub(NULL), out_sub(NULL), messages_(0), message_(0)
{
    addTag(Tag::get("General"));

    current_result_.reset(new VectorMessage);
}

Foreach::~Foreach()
{
    if(in_sub){
        in_sub->deleteLater();
    }
    if(out_sub) {
        out_sub->deleteLater();
    }
}

void Foreach::process()
{
    VectorMessage::Ptr vec = input_->getMessage<VectorMessage>();

    in_sub->waitForProcessing();

    out_sub->setType(vec->getSubType());

    messages_ = vec->value.size();

    for(int i = 0; i < messages_; ++i) {
        std::stringstream step; step << "step " << i;
        Timer::Interlude::Ptr interlude = publish_timer_->step(step.str());
        out_sub->publish(vec->value[i]);
    }
}


void Foreach::appendMessageFrom(Connectable *)
{

    ConnectorOut* out = dynamic_cast<ConnectorOut*>(in_sub->getSource());
    if(!out) {
        return;
    }
    ConnectionType::Ptr msg = out->getMessage();

    current_result_->value.push_back(msg);

    ++message_;
    if(message_ >= messages_) {
        message_ = 0;

        output_->setType(current_result_);
        output_->publish(current_result_);

        current_result_.reset(new VectorMessage);
    }

    in_sub->setProcessing(false);
}

Connectable* Foreach::getConnector(const UUID &uuid) const
{
    Connectable* p = Node::getConnector(uuid);
    if(!p) {
        if(uuid == out_sub->getUUID()) {
            return out_sub;

        } else if(uuid == in_sub->getUUID()) {
            return in_sub;

        } else {
            return NULL;
        }
    }

    return p;
}


ConnectorIn* Foreach::getInput(const UUID& uuid) const
{
    if(in_sub->getUUID() == uuid) {
        return in_sub;
    }
    return Node::getInput(uuid);
}

ConnectorOut* Foreach::getOutput(const UUID& uuid) const
{
    if(out_sub->getUUID() == uuid) {
        return out_sub;
    }
    return Node::getOutput(uuid);
}

void Foreach::stop()
{
    QObject::disconnect(in_sub);

    in_sub->stop();
    out_sub->stop();
    disconnectConnector(in_sub);
    disconnectConnector(out_sub);

    Node::stop();
}

std::vector<ConnectorOut*> Foreach::getOutputs() const
{
    std::vector<ConnectorOut*> output = Node::getOutputs();
    if(out_sub) {
        output.push_back(out_sub);
    }

    return output;
}

std::vector<ConnectorIn*> Foreach::getInputs() const
{
    std::vector<ConnectorIn*> input = Node::getInputs();
    if(in_sub) {
        input.push_back(in_sub);
    }

    return input;
}

void Foreach::fill(QBoxLayout *layout)
{
    setSynchronizedInputs(true);

    input_ = addInput<VectorMessage>("Vector");

    output_ = addOutput<VectorMessage>("Content");

    QHBoxLayout* sub = new QHBoxLayout;

    out_sub = new ConnectorOut(getSettings(), UUID::make_sub(getUUID(), "out_sub"));
    in_sub = new ConnectorIn(getSettings(), UUID::make_sub(getUUID(), "in_sub"));

    out_sub->setType(AnyMessage::make());
    in_sub->setType(AnyMessage::make());

    sub->addWidget(new Port(getCommandDispatcher(), out_sub));
    sub->addWidget(new Port(getCommandDispatcher(), in_sub));

    out_sub->enable();
    in_sub->enable();

    layout->addLayout(sub);

    QObject::connect(in_sub, SIGNAL(messageArrived(Connectable*)), this, SLOT(appendMessageFrom(Connectable*)), Qt::DirectConnection);
}
