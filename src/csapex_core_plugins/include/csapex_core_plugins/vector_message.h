#ifndef VECTOR_MESSAGE_H
#define VECTOR_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <string>

namespace csapex {
namespace connection_types {

template <class M>
struct VectorMessage : public MessageTemplate<std::vector<typename M::Ptr>, VectorMessage<M> >
{
    typedef MessageTemplate<std::vector<typename M::Ptr>, VectorMessage<M> > Parent;
    typedef typename Parent::Ptr Ptr;
    typedef std::vector<typename M::Ptr> Vector;

    VectorMessage()
        : MessageTemplate<Vector, VectorMessage<M> > ("std::vector<?>")
    {
        M tmp;
        Parent::setName(std::string("std::vector<") + tmp.rawName()  + "::Ptr>");
    }

    static ConnectionType::Ptr make(){
        Ptr new_msg(new VectorMessage<M>);
        return new_msg;
    }

    void writeYaml(YAML::Emitter& yaml) {
        yaml << YAML::Key << "value" << YAML::Value << YAML::Flow;
        yaml << YAML::BeginSeq;
        for(unsigned i = 0; i < Parent::value.size(); ++i) {
            yaml << YAML::BeginMap;
            typename M::Ptr mptr = Parent::value[i];
            mptr->writeYaml(yaml);
            yaml << YAML::EndMap;
        }
        yaml << YAML::EndSeq;
    }
    void readYaml(const YAML::Node& node) {
        if(node.FindValue("value")) {
            assert(node["value"].Type() == YAML::NodeType::Sequence);
            for(YAML::Iterator it = node["value"].begin(); it != node["value"].end(); ++it) {
                //*it >> Parent::value;
                typename M::Ptr e(new M);
                e->readYaml(*it);
                Parent::value.push_back(e);
            }
        }
    }

};

}
}

#endif // VECTOR_MESSAGE_H
