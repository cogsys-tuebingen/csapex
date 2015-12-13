#ifndef YAML_NODE_BUILDER_H
#define YAML_NODE_BUILDER_H

/// SYSTEM
#include <yaml-cpp/yaml.h>

#include <yaml-cpp/anchor.h>
#include <yaml-cpp/contrib/anchordict.h>
#include <yaml-cpp/contrib/graphbuilder.h>
#include <yaml-cpp/eventhandler.h>
#include <vector>
#include "yaml-cpp/node/ptr.h"

namespace YAML {
class GraphBuilderInterface;
struct Mark;


namespace detail {
class node;
}  // namespace detail
struct Mark;
}  // namespace YAML

namespace YAML {
class Node;

class NodeBuilder : public EventHandler {
 public:
  NodeBuilder();
  virtual ~NodeBuilder();

  Node Root();

  virtual void OnDocumentStart(const Mark& mark) override;
  virtual void OnDocumentEnd() override;

  virtual void OnNull(const Mark& mark, anchor_t anchor) override;
  virtual void OnAlias(const Mark& mark, anchor_t anchor) override;
  virtual void OnScalar(const Mark& mark, const std::string& tag,
                        anchor_t anchor, const std::string& value) override;

  virtual void OnSequenceStart(const Mark& mark, const std::string& tag,
                               anchor_t anchor) override;
  virtual void OnSequenceEnd() override;

  virtual void OnMapStart(const Mark& mark, const std::string& tag,
                          anchor_t anchor) override;
  virtual void OnMapEnd() override;

 private:
  detail::node& Push(anchor_t anchor);
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

}

#endif // YAML_NODE_BUILDER_H
