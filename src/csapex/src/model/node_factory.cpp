/// HEADER
#include <csapex/model/node_factory.h>

/// COMPONENT
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/tag.h>
#include <csapex/utility/uuid.h>
#include <csapex/plugin/plugin_manager.hpp>

/// SYSTEM

#include <boost/algorithm/string.hpp>

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

NodeFactory::NodeFactory(Settings &settings, csapex::PluginLocator* locator)
    : settings_(settings), plugin_locator_(locator),
      node_manager_(new PluginManager<Node> ("csapex::Node")),
      tag_map_has_to_be_rebuilt_(false)
{
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
    delete node_manager_;
    node_manager_ = nullptr;
}


void NodeFactory::loadPlugins()
{
    ensureLoaded();
    
    rebuildMap();
}

void NodeFactory::rebuildPrototypes()
{
//    available_elements_prototypes.clear();
//    node_adapter_builders_.clear();
    
    typedef std::pair<std::string, PluginConstructor<Node> > NODE_PAIR;
    for(const NODE_PAIR& p : node_manager_->availableClasses()) {
        const PluginConstructor<Node>& plugin_constructor = p.second;

        // convert tag list into vector
        std::vector<std::string> tokens;
        std::vector<Tag::Ptr> tags;

        std::string taglist = p.second.getTags();
        boost::algorithm::split(tokens, taglist, boost::is_any_of(",;"));

        for(std::vector<std::string>::const_iterator it = tokens.begin(); it != tokens.end(); ++it) {
            std::string str = boost::algorithm::trim_copy(*it);
            if(!str.empty()) {
                tags.push_back(Tag::get(str));
            }
        }

        // make the constructor
        csapex::NodeConstructor::Ptr constructor(new csapex::NodeConstructor(
                                                     settings_,
                                                     p.second.getType(), p.second.getDescription(),
                                                     p.second.getIcon(),
                                                     tags,
                                                     plugin_constructor));

        plugin_constructor.unload_request->connect(*constructor->unload_request);
        plugin_constructor.reload_request->connect(*constructor->reload_request);

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
                tag_map_[tag].push_back(p);
                has_tag = true;
            }
            
            if(!has_tag) {
                tag_map_[general].push_back(p);
            }
            
            ++it;
            
        } catch(const NodeConstructor::NodeConstructionException& e) {
            std::cerr << "warning: cannot load node: " << e.what() << std::endl;
            it = constructors_.erase(it);
        }
    }
    

    typedef std::map<TagPtr, std::vector<NodeConstructor::Ptr> > map;
    for(map::iterator it = tag_map_.begin(); it != tag_map_.end(); ++it) {
        std::sort(it->second.begin(), it->second.end(), compare);
    }
    
    tag_map_has_to_be_rebuilt_ = false;
}

std::map<TagPtr, std::vector<NodeConstructor::Ptr> > NodeFactory::getTagMap()
{
    ensureLoaded();
    return tag_map_;
}

void NodeFactory::ensureLoaded()
{
    if(!node_manager_->pluginsLoaded()) {
        node_manager_->load(plugin_locator_);
        
        rebuildPrototypes();
        
        tag_map_has_to_be_rebuilt_ = true;
    }
    
    if(tag_map_has_to_be_rebuilt_) {
        rebuildMap();
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

NodeWorkerPtr NodeFactory::makeSingleNode(NodeConstructor::Ptr content, const UUID& uuid)
{
    return content->makeNodeWorker(uuid);
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

NodeWorkerPtr NodeFactory::makeNode(const std::string& target_type, const UUID& uuid)
{
    return makeNode(target_type, uuid, nullptr);
}

NodeWorkerPtr NodeFactory::makeNode(const std::string& target_type, const UUID& uuid, NodeStatePtr state)
{
    apex_assert_hard(!uuid.empty());

    NodeConstructorPtr p = getConstructor(target_type);
    if(p) {
        NodeWorkerPtr result = makeSingleNode(p, uuid);

        if(state) {
            result->setNodeState(state);
        }

        reload_connections_[uuid] = p->unload_request->connect(std::bind(&NodeFactory::unloadNode, this, p, uuid));

        return result;

    } else {
        std::cerr << "error: cannot make node, type '" << target_type << "' is unknown" << std::endl;
        return nullptr;
    }
}

void NodeFactory::unloadNode(NodeConstructorPtr p, UUID uuid)
{
    reload_connections_[uuid].disconnect();
    reload_connections_[uuid] = p->reload_request->connect(std::bind(&NodeFactory::reloadNode, this, p, uuid));

    unload_request(uuid);
}


void NodeFactory::reloadNode(NodeConstructorPtr /*p*/, UUID uuid)
{
    reload_connections_[uuid].disconnect();

    reload_request(uuid);
}
