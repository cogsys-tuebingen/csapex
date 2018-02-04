/// HEADER
#include <csapex/factory/node_factory_impl.h>

/// COMPONENT
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/direct_node_worker.h>
#include <csapex/model/subprocess_node_worker.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_runner.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/node_facade_proxy.h>
#include <csapex/model/tag.h>
#include <csapex/utility/uuid.h>
#include <csapex/plugin/plugin_manager.hpp>
#include <csapex/model/subgraph_node.h>
#include <csapex/nodes/sticky_note.h>
#include <csapex/param/string_list_parameter.h>
#include <csapex/model/graph/graph_impl.h>

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

NodeFactoryImplementation::NodeFactoryImplementation(Settings& settings, PluginLocator* locator)
    : settings_(settings), plugin_locator_(locator),
      node_manager_(std::make_shared<PluginManager<Node>> ("csapex::Node")),
      tag_map_has_to_be_rebuilt_(false)
{
    NodeConstructorPtr provider = std::make_shared<NodeConstructor>("csapex::Graph", []{
        GraphImplementationPtr graph = std::make_shared<GraphImplementation>();
        return std::make_shared<SubgraphNode>(graph);
    });
    provider->setIcon(":/group.png");
    registerNodeType(provider, true);

    NodeConstructorPtr note = std::make_shared<NodeConstructor>("csapex::Note", []{
        return std::make_shared<Note>();
    });
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


void NodeFactoryImplementation::loadPlugins()
{
    ensureLoaded();

    rebuildMap();
}

void NodeFactoryImplementation::shutdown()
{
    tag_map_.clear();
    constructors_.clear();
}

void NodeFactoryImplementation::rebuildPrototypes()
{
    settings_.setQuiet(true);

    bool dirty = false;

    int plugin_count = 0;
    for(const auto& p : node_manager_->getConstructors()) {
        const PluginConstructor<Node>& plugin_constructor = p.second;

        // make the constructor
        auto type = p.second.getType();
        csapex::NodeConstructor::Ptr constructor = std::make_shared<NodeConstructor>(type, plugin_constructor);

        constructor->setDescription(p.second.getDescription()).setIcon(p.second.getIcon()).setTags(p.second.getTags());

        // set cached properties, if the library modification time has not changed.
        // otherwise rely on lazy initialization
        std::string mod = std::string("__last_mod_") + type;
        std::string key = std::string("__cached_properties_") + type;
        bool loaded = false;
        if(settings_.knows(mod)) {
            long last_mod = settings_.get<long>(mod);
            if(last_mod == node_manager_->getLastModification(type)) {
                param::StringListParameter::Ptr properties = std::dynamic_pointer_cast<param::StringListParameter>(settings_.get(key));
                if(properties) {
                    constructor->setProperties(properties->getValues());
                    loaded = true;
                }
            }
        }

        if(!loaded) {
            NOTIFICATION_INFO("reloading properties for node type " << type);
            dirty = true;
            try {
                auto last_modification = node_manager_->getLastModification(type);

                param::StringListParameter::Ptr properties(new param::StringListParameter(key, param::ParameterDescription()));
                properties->set(constructor->getProperties());
                settings_.addPersistent(properties);
                settings_.setPersistent(mod, last_modification);

            } catch(const std::exception& e) {
                NOTIFICATION("plugin '" << type << "' cannot be loaded");
            }
        }

        registerNodeType(constructor, true);
        ++plugin_count;
    }

    std::cout << "loaded " << plugin_count << " plugins" << std::endl;

    settings_.setQuiet(false);

    if(dirty) {
        settings_.savePersistent();
    }
}

void NodeFactoryImplementation::rebuildMap()
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
            NOTIFICATION("cannot load node: " << e.what());
            it = constructors_.erase(it);
        }
    }


    typedef std::map<std::string, std::vector<NodeConstructor::Ptr> > map;
    for(map::iterator it = tag_map_.begin(); it != tag_map_.end(); ++it) {
        std::sort(it->second.begin(), it->second.end(), compare);
    }

    tag_map_has_to_be_rebuilt_ = false;
}

void NodeFactoryImplementation::ensureLoaded()
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


void NodeFactoryImplementation::registerNodeType(NodeConstructor::Ptr provider, bool suppress_signals)
{
    constructors_.push_back(provider);
    tag_map_has_to_be_rebuilt_ = true;

    if(!suppress_signals) {
        new_node_type();
    }
}



NodeFacadeImplementationPtr NodeFactoryImplementation::makeNode(const std::string& target_type,
                                                                const UUID& uuid,
                                                                const UUIDProviderPtr& uuid_provider,
                                                                const ExecutionType exec_type)
{
    return makeNode(target_type, uuid, uuid_provider, exec_type, nullptr);
}

NodeFacadeImplementationPtr NodeFactoryImplementation::makeNode(const std::string& target_type,
                                                                const UUID& uuid,
                                                                const UUIDProviderPtr& uuid_provider,
                                                                const ExecutionType default_exec_type,
                                                                NodeStatePtr state)
{
    NodeConstructorPtr p = getConstructor(target_type);
    if(p) {
        NodeHandlePtr nh = p->makeNodeHandle(uuid, uuid_provider);
        if(!nh) {
            NOTIFICATION("error: cannot make node of type '" << target_type);
            return nullptr;
        }

        ExecutionType exec_type = state ? state->getExecutionType() : default_exec_type;

        NodeFacadeImplementationPtr result;
        NodeWorkerPtr nw;
        if(!nh->isIsolated()) {

            switch(exec_type) {
            case ExecutionType::AUTO:
            case ExecutionType::DIRECT:
                nw = std::make_shared<DirectNodeWorker>(nh);
                break;
            case ExecutionType::SUBPROCESS:
                nw = std::make_shared<SubprocessNodeWorker>(nh);
                break;
            }

            NodeRunnerPtr runner = std::make_shared<NodeRunner>(nw);
            result = std::make_shared<NodeFacadeImplementation>(nh, nw, runner);


        } else {
            result = std::make_shared<NodeFacadeImplementation>(nh);
        }

        NodePtr node = result->getNode();
        try {
            node->setupParameters(*node);
            node->setup(*nh);

        } catch(const std::exception& e) {
            node->aerr << "setup failed: " << e.what() << std::endl;
        }

        if(nw) {
            nw->initialize();
        }
        // TODO: can this be done more elegantly?
        SubgraphNodePtr subgraph = std::dynamic_pointer_cast<SubgraphNode>(nh->getNode().lock());
        if(subgraph) {
            apex_assert_hard(subgraph);
            subgraph->setNodeFacade(result);
        }

        if(state) {
            nh->setNodeState(state);
        }

        nh->getNodeState()->setExecutionType(exec_type);

        node_constructed(result);

        return result;

    } else {
        NOTIFICATION("error: cannot make node, type '" << target_type << "' is unknown");
        return nullptr;
    }
}

NodeFacadeImplementationPtr NodeFactoryImplementation::makeGraph(const UUID& uuid, const UUIDProviderPtr& uuid_provider)
{
    return makeNode("csapex::Graph", uuid, uuid_provider);
}

NodeFacadeImplementationPtr NodeFactoryImplementation::makeGraph(const UUID& uuid, const UUIDProviderPtr& uuid_provider, NodeStatePtr state)
{
    return makeNode("csapex::Graph", uuid, uuid_provider, ExecutionType::DIRECT, state);
}
