/// HEADER
#include <csapex/factory/node_factory.h>

/// COMPONENT
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/tag.h>
#include <csapex/utility/uuid.h>
#include <csapex/plugin/plugin_manager.hpp>
#include <csapex/model/subgraph_node.h>
#include <csapex/nodes/note.h>

using namespace csapex;


namespace csapex
{
template <>
struct PluginManagerGroup<Node>
{
    enum Group {
        value = 0
    };
};
}

NodeFactory::NodeFactory(csapex::PluginLocator* locator)
    : plugin_locator_(locator),
      node_manager_(std::make_shared<PluginManager<Node>> ("csapex::Node")),
      tag_map_has_to_be_rebuilt_(false)
{
    NodeConstructorPtr provider = std::make_shared<NodeConstructor>("csapex::Graph", []{ return std::make_shared<SubgraphNode>(); });
    provider->setIcon(":/group.png");
    registerNodeType(provider, true);

    NodeConstructorPtr note = std::make_shared<NodeConstructor>("csapex::Note", []{ return std::make_shared<Note>(); });
    note->setIcon(":/note.png");
    note->setDescription("A sticky note to keep information.");
    registerNodeType(note, true);

    node_manager_->manifest_loaded.connect(manifest_loaded);
}

namespace {
bool compare (NodeConstructor::Ptr a, NodeConstructor::Ptr b) {
    const std::string& as = UUID::stripNamespace(a->getType());
    const std::string& bs = UUID::stripNamespace(b->getType());
    return as.compare(bs) < 0;
}
}


NodeFactory::~NodeFactory()
{
}


void NodeFactory::loadPlugins()
{
    ensureLoaded();
    
    rebuildMap();
}

void NodeFactory::shutdown()
{
    tag_map_.clear();
    constructors_.clear();
}

void NodeFactory::rebuildPrototypes()
{
    //    available_elements_prototypes.clear();
    //    node_adapter_builders_.clear();
    
    for(const auto& p : node_manager_->getConstructors()) {
        const PluginConstructor<Node>& plugin_constructor = p.second;

        // make the constructor
        csapex::NodeConstructor::Ptr constructor = std::make_shared<NodeConstructor>(p.second.getType(), plugin_constructor);

        constructor->setDescription(p.second.getDescription()).setIcon(p.second.getIcon()).setTags(p.second.getTags());

        registerNodeType(constructor, true);
    }
}

void NodeFactory::rebuildMap()
{
    Tag::createIfNotExists("General");
    Tag::Ptr general = Tag::get("General");
    
    tag_map_.clear();
    
    for(std::vector<NodeConstructor::Ptr>::iterator
        it = constructors_.begin();
        it != constructors_.end();) {
        
        const NodeConstructor::Ptr& p = *it;
        
        try {
            bool has_tag = false;
            for(const Tag::Ptr& tag : p->getTags()) {
                tag_map_[tag->getName()].push_back(p);
                has_tag = true;
            }
            
            if(!has_tag) {
                tag_map_[general->getName()].push_back(p);
            }
            
            ++it;
            
        } catch(const NodeConstructor::NodeConstructionException& e) {
            std::cerr << "warning: cannot load node: " << e.what() << std::endl;
            it = constructors_.erase(it);
        }
    }
    

    typedef std::map<std::string, std::vector<NodeConstructor::Ptr> > map;
    for(map::iterator it = tag_map_.begin(); it != tag_map_.end(); ++it) {
        std::sort(it->second.begin(), it->second.end(), compare);
    }
    
    tag_map_has_to_be_rebuilt_ = false;
}

std::map<std::string, std::vector<NodeConstructor::Ptr> > NodeFactory::getTagMap()
{
    ensureLoaded();
    return tag_map_;
}

void NodeFactory::ensureLoaded()
{
    if(plugin_locator_) {
        if(!node_manager_->pluginsLoaded()) {
            node_manager_->load(plugin_locator_);

            rebuildPrototypes();

            tag_map_has_to_be_rebuilt_ = true;
        }

        if(tag_map_has_to_be_rebuilt_) {
            rebuildMap();
        }
    }
}


void NodeFactory::registerNodeType(NodeConstructor::Ptr provider, bool suppress_signals)
{
    constructors_.push_back(provider);
    tag_map_has_to_be_rebuilt_ = true;
    
    if(!suppress_signals) {
        new_node_type();
    }
}

bool NodeFactory::isValidType(const std::string &type) const
{
    for(NodeConstructor::Ptr p : constructors_) {
        if(p->getType() == type) {
            return true;
        }
    }
    
    return false;
}

NodeConstructor::Ptr NodeFactory::getConstructor(const std::string &target_type)
{
    ensureLoaded();

    std::string type = target_type;
    if(type.find_first_of(" ") != type.npos) {
        std::cout << "warning: type '" << type << "' contains spaces, stripping them!" << std::endl;
        while(type.find(" ") != type.npos) {
            type.replace(type.find(" "), 1, "");
        }
    }

    for(NodeConstructor::Ptr p : constructors_) {
        if(p->getType() == type) {
            return p;
        }
    }

    // cannot make box, type is unknown, trying different namespace
    std::string type_wo_ns = UUID::stripNamespace(type);

    for(NodeConstructor::Ptr p : constructors_) {
        std::string p_type_wo_ns = UUID::stripNamespace(p->getType());

        if(p_type_wo_ns == type_wo_ns) {
            return p;
        }
    }

    return nullptr;
}

std::vector<NodeConstructorPtr> NodeFactory::getConstructors()
{
    ensureLoaded();

    return constructors_;
}

NodeHandlePtr NodeFactory::makeNode(const std::string& target_type, const UUID& uuid, UUIDProvider *uuid_provider)
{
    return makeNode(target_type, uuid, uuid_provider, nullptr);
}

NodeHandlePtr NodeFactory::makeNode(const std::string& target_type, const UUID& uuid, UUIDProvider *uuid_provider,  NodeStatePtr state)
{
    apex_assert_hard(target_type == "csapex::Graph" || !uuid.empty());

    NodeConstructorPtr p = getConstructor(target_type);
    if(p) {
        NodeHandlePtr result = p->makeNodeHandle(uuid, uuid_provider);

        if(state) {
            result->setNodeState(state);
        }

        return result;

    } else {
        std::cerr << "error: cannot make node, type '" << target_type << "' is unknown" << std::endl;
        return nullptr;
    }
}
