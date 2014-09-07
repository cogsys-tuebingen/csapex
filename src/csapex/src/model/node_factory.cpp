/// HEADER
#include <csapex/model/node_factory.h>

/// COMPONENT
#include <csapex/command/delete_node.h>
#include <csapex/command/meta.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/tag.h>
#include <csapex/utility/uuid.h>
#include <csapex/view/box.h>
#include <csapex/view/default_node_adapter.h>
#include <csapex/utility/plugin_manager.hpp>
#include <csapex/view/widget_controller.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <stack>
#include <qmime.h>
#include <boost/algorithm/string.hpp>

using namespace csapex;

NodeFactory::NodeFactory(Settings &settings)
    : settings_(settings),
      node_manager_(new PluginManager<Node> ("csapex::Node")),
      node_adapter_manager_(new PluginManager<NodeAdapterBuilder> ("csapex::NodeAdapterBuilder")),
      dirty_(false)
{
    node_manager_->loaded.connect(loaded);
    node_adapter_manager_->loaded.connect(loaded);
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
    node_manager_ = NULL;
    
    node_adapter_builders_.clear();
    
    delete node_adapter_manager_;
    node_adapter_manager_ = NULL;
}


void NodeFactory::loadPlugins()
{
    node_manager_->load();
    node_adapter_manager_->load();
    rebuildPrototypes();
    
    rebuildMap();
}

void NodeFactory::rebuildPrototypes()
{
//    available_elements_prototypes.clear();
//    node_adapter_builders_.clear();
    
    typedef std::pair<std::string, DefaultConstructor<Node> > NODE_PAIR;
    Q_FOREACH(const NODE_PAIR& p, node_manager_->availableClasses()) {
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
                                                     p.second));
        registerNodeType(constructor, true);
    }
    
    typedef std::pair<std::string, DefaultConstructor<NodeAdapterBuilder> > ADAPTER_PAIR;
    Q_FOREACH(const ADAPTER_PAIR& p, node_adapter_manager_->availableClasses()) {
        NodeAdapterBuilder::Ptr builder = p.second.construct();
        node_adapter_builders_[builder->getWrappedType()] = builder;
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
            Q_FOREACH(const Tag::Ptr& tag, p->getTags()) {
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
    
    dirty_ = false;
}

std::map<TagPtr, std::vector<NodeConstructor::Ptr> > NodeFactory::getTagMap()
{
    ensureLoaded();
    return tag_map_;
}

void NodeFactory::ensureLoaded()
{
    if(!node_manager_->pluginsLoaded()) {
        node_manager_->load();
        node_adapter_manager_->load();
        
        rebuildPrototypes();
        
        dirty_ = true;
    }
    
    if(dirty_) {
        rebuildMap();
    }
}


void NodeFactory::registerNodeType(NodeConstructor::Ptr provider, bool suppress_signals)
{
    constructors_.push_back(provider);
    dirty_ = true;
    
    if(!suppress_signals) {
        new_node_type();
    }
}

bool NodeFactory::isValidType(const std::string &type) const
{
    Q_FOREACH(NodeConstructor::Ptr p, constructors_) {
        if(p->getType() == type) {
            return true;
        }
    }
    
    return false;
}

Node::Ptr NodeFactory::makeSingleNode(NodeConstructor::Ptr content, const UUID& uuid)
{
    return content->makeContent(uuid);
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

    BOOST_FOREACH(NodeConstructor::Ptr p, constructors_) {
        if(p->getType() == type) {
            return p;
        }
    }

    // cannot make box, type is unknown, trying different namespace
    std::string type_wo_ns = UUID::stripNamespace(type);

    BOOST_FOREACH(NodeConstructor::Ptr p, constructors_) {
        std::string p_type_wo_ns = UUID::stripNamespace(p->getType());

        if(p_type_wo_ns == type_wo_ns) {
            return p;
        }
    }

    return NodeConstructorNullPtr;
}

std::vector<NodeConstructorPtr> NodeFactory::getConstructors()
{
    ensureLoaded();

    return constructors_;
}

Node::Ptr NodeFactory::makeNode(const std::string& target_type, const UUID& uuid)
{
    return makeNode(target_type, uuid, NodeStateNullPtr);
}

Node::Ptr NodeFactory::makeNode(const std::string& target_type, const UUID& uuid, NodeStatePtr state)
{
    apex_assert_hard(!uuid.empty());

    NodeConstructorPtr p = getConstructor(target_type);
    if(p) {
        Node::Ptr result = makeSingleNode(p, uuid);

        if(state) {
            result->setNodeState(state);
        }

        return result;

    } else {
        std::cerr << "error: cannot make node, type '" << target_type << "' is unknown" << std::endl;
        return NodeNullPtr;
    }
}
