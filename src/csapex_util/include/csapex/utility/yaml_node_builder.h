#ifndef YAML_NODE_BUILDER_H
#define YAML_NODE_BUILDER_H

/// PROJECT
#include <csapex_util/export.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/anchor.h>
#include <yaml-cpp/contrib/anchordict.h>
#include <yaml-cpp/contrib/graphbuilder.h>
#include <yaml-cpp/eventhandler.h>
#include <vector>
#include "yaml-cpp/node/ptr.h"

#if defined(EMITTERSTYLE_H_62B23520_7C8E_11DE_8A39_0800200C9A66)
#define USE_YAML_EMITTER_STYLE 1
#else
#define USE_YAML_EMITTER_STYLE 0
#endif

namespace YAML
{
namespace detail
{
class node;
}  // namespace detail
struct Mark;
}  // namespace YAML

namespace YAML
{
class Node;

class NodeBuilder : public EventHandler
{
public:
    NodeBuilder();
    ~NodeBuilder() override;

    Node Root();

    void OnDocumentStart(const Mark& mark) override;
    void OnDocumentEnd() override;

    void OnNull(const Mark& mark, anchor_t anchor) override;
    void OnAlias(const Mark& mark, anchor_t anchor) override;
    void OnScalar(const Mark& mark, const std::string& tag, anchor_t anchor, const std::string& value) override;

    void OnSequenceStart(const Mark& mark, const std::string& tag, anchor_t anchor
#if USE_YAML_EMITTER_STYLE
                         ,
                         EmitterStyle::value style) override;
#else
                         ) override;
#endif

    void OnSequenceEnd() override;

    void OnMapStart(const Mark& mark, const std::string& tag, anchor_t anchor
#if USE_YAML_EMITTER_STYLE
                    ,
                    EmitterStyle::value style) override;
#else
                    ) override;
#endif
    void OnMapEnd() override;

private:
    detail::node& Push(const Mark& mark, anchor_t anchor);
    void Push(detail::node& node);
    void Pop();
    void RegisterAnchor(anchor_t anchor, detail::node& node);

private:
    detail::shared_memory_holder m_pMemory;
    detail::node* m_pRoot;

    typedef std::vector<detail::node*> Nodes;
    Nodes m_stack;
    Nodes m_anchors;

    typedef std::pair<detail::node*, bool> PushedKey;
    std::vector<PushedKey> m_keys;
    std::size_t m_mapDepth;
};
}  // namespace YAML
#endif  // YAML_NODE_BUILDER_H
