/// HEADER
#include <csapex/model/throttled_node.h>

/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/model/token.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>

using namespace csapex;
using namespace csapex::connection_types;

ThrottledNode::ThrottledNode()
{
}

void ThrottledNode::setupParameter(csapex::Parameterizable& params, const std::string& name)
{
    params.addParameter(param::ParameterFactory::declareRange(name, 0.1, 100.0, 10.0, 0.1), [this](param::Parameter* p) { node_handle_->getNodeState()->setMaximumFrequency(p->as<double>()); });
    node_handle_->getNodeState()->max_frequency_changed.connect([name, this]() {
        double max_f = node_handle_->getNodeState()->getMaximumFrequency();
        setParameter(name, max_f);
    });
}
