/// HEADER
#include "foreach.h"

/// COMPONENT
#include <csapex_core_plugins/vector_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex/view/port.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Foreach, csapex::Node)

using namespace csapex;
using namespace connection_types;

Foreach::Foreach()
    : in_sub(NULL), out_sub(NULL)
{
    addTag(Tag::get("General"));
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

void Foreach::allConnectorsArrived()
{
    VectorMessage::Ptr vec = input_->getMessage<VectorMessage>();
    //if(!current_result_) {
        current_result_.reset(new VectorMessage);
    //}

    out_sub->setType(vec->getSubType());

    for(int i = 0, n = vec->value.size(); i < n; ++i) {
        out_sub->publish(vec->value[i]);
    }
//    out_sub->publish(vec->value[rand() % vec->value.size()]);

    std::cerr << "publishing vector of size " << current_result_->value.size() << std::endl;
    output_->setType(current_result_);
    output_->publish(current_result_);
}


void Foreach::appendMessageFrom(Connectable *)
{
    ConnectorOut* out = dynamic_cast<ConnectorOut*>(in_sub->getSource());
    if(!out) {
        return;
    }
    ConnectionType::Ptr msg = out->getMessage();

    current_result_->value.push_back(msg);

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
    output.push_back(out_sub);

    return output;
}

std::vector<ConnectorIn*> Foreach::getInputs() const
{
    std::vector<ConnectorIn*> input = Node::getInputs();
    input.push_back(in_sub);

    return input;
}

void Foreach::fill(QBoxLayout *layout)
{
    setSynchronizedInputs(true);

    input_ = addInput<VectorMessage>("Vector");

    output_ = addOutput<VectorMessage>("Content");

    QHBoxLayout* sub = new QHBoxLayout;

    out_sub = new ConnectorOut(UUID::make_sub(getUUID(), "out_sub"));
    in_sub = new ConnectorIn(UUID::make_sub(getUUID(), "in_sub"));

    out_sub->setType(AnyMessage::make());
    in_sub->setType(AnyMessage::make());

    sub->addWidget(new Port(getCommandDispatcher(), out_sub));
    sub->addWidget(new Port(getCommandDispatcher(), in_sub));

    out_sub->enable();
    in_sub->enable();

    layout->addLayout(sub);

    QObject::connect(in_sub, SIGNAL(messageArrived(Connectable*)), this, SLOT(appendMessageFrom(Connectable*)));
}
