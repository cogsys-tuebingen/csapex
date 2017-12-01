/// HEADER
#include <csapex/factory/node_factory_impl.h>

using namespace csapex;

NodeConstructor::Ptr NodeFactory::getConstructor(const std::string &target_type)
{
    ensureLoaded();

    std::string type = target_type;
    if(type.find_first_of(" ") != type.npos) {
        NOTIFICATION_WARN("type '" << type << "' contains spaces, stripping them!");
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
std::map<std::string, std::vector<NodeConstructorPtr> > NodeFactory::getTagMap()
{
    ensureLoaded();

    return tag_map_;
}


bool NodeFactory::isValidType(const std::string &type)
{
    ensureLoaded();

    for(NodeConstructor::Ptr p : constructors_) {
        if(p->getType() == type) {
            return true;
        }
    }
    return false;
}
