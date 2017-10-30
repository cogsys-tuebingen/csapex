/// HEADER
#include <csapex/factory/node_factory_remote.h>

/// PROJECT
#include <csapex/io/protcol/request_nodes.h>
#include <csapex/model/node_constructor.h>

using namespace csapex;

NodeFactoryRemote::NodeFactoryRemote(const SessionPtr& session)
    : Remote(session)
{
}

void NodeFactoryRemote::ensureLoaded()
{
    if(tag_map_.empty()) {
        if(const auto& response = session_->sendRequest<RequestNodes>()) {
            std::map<std::string, NodeConstructorPtr> temp_mapping;

            // build node constructor list
            for(const auto& pair : response->getTagMap()) {
                const std::string& tag = pair.first;
                for(const NodeConstructorPtr& constructor : pair.second) {

                    auto pos = temp_mapping.find(constructor->getType());
                    if(pos == temp_mapping.end()) {
                        constructors_.push_back(constructor);
                        temp_mapping.insert(std::make_pair(constructor->getType(), constructor));
                    }

                    tag_map_[tag].push_back(temp_mapping[constructor->getType()]);
                }
            }
        }
    }
}
