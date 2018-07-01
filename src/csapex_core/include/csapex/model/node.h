#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/model/parameterizable.h>
#include <csapex/utility/stream_relay.h>
#include <csapex/utility/assert.h>
#include <csapex/profiling/timable.h>
#include <csapex_core/csapex_core_export.h>
#include <csapex/utility/export_plugin.h>
#include <csapex/model/observer.h>

namespace csapex
{
/**
 * @typedef ProcessingFunction
 * @brief Represents a call to a process-like function, having the same  signature as Node::process.
 */
using ProcessingFunction = std::function<void(csapex::NodeModifier&, Parameterizable&)>;

/**
 * @typedef Continuation
 * @brief A continuation is a function that returns the control state back to the caller.
 *
 * This is specifically used with nodes, so the control state will return to the process calling Node::process.
 * A continuation C is of the form
 *   C( Fn )
 * where Fn is a ProcessingFunction.
 *
 * Fn will be called in the context of the callee of Node::process, returning the control.
 */
using Continuation = std::function<void(ProcessingFunction)>;

/**
 * @brief Node is the most fundamental base class of the frame work. Plug-ins mostly implement this interface.
 *
 * A node represents a single data processing unit in the computation graph.
 * Multiple instances of a node can be active at the same time, so no static data should be used.
 * Nodes read data sent via synchronous message passing on input ports and send data on output ports.
 * Slots and events can be used to process messages asynchronously.
 * Nodes can be activiated externally, which can also be handled in implementation classes.
 */
class CSAPEX_CORE_EXPORT Node : public Parameterizable, public Timable, public Observer
{
public:
    /**
     * @typedef Ptr
     * Nested shared pointer typedef
     */
    typedef std::shared_ptr<Node> Ptr;

public:
    /**
     * @brief Node
     * A default constructor is necessary for plug-in based system.
     */
    Node();

    /**
     * @brief ~Node
     */
    virtual ~Node();

    /**
     * @brief initialize has to be called before the node is used.
     *
     * This function replaces the constructor and is necessary because plug-ins have to
     * be created with the default constructor.
     *
     * @param node_handle is the associated node handle for the node instance
     */
    virtual void initialize(NodeHandlePtr node_handle);

    /**
     * @brief detach removes the node from its node handle.
     */
    virtual void detach();

    /**
     * @brief getNodeHandle gives access to the corresponding node handle.
     * @return to the corresponding node handle
     */
    NodeHandle* getNodeHandle() const;

    /**
     * @brief getUUID is a shorthand accessor for the UUID of the corresponding node handle
     * @return the node's unique identifier
     */
    UUID getUUID() const;

public:
    /**
     * @brief setupParameters is used to specify the parameters of a node.
     *
     * The method should terminate quickly and should not block,
     * as it is called to determine the properties of the node.
     *
     * @param parameters The object holding all parameters.
     */
    virtual void setupParameters(Parameterizable& parameters);

    /**
     * @brief setup is used to specify ingoing and outgoing ports.
     *
     * The method is called before the node is executed the first time and after
     * the parameters are set up. After the call has returned, the system assumes
     * that the node is ready to process data.
     *
     * @param node_modifier The node modifier this node can use to modify itself.
     */
    virtual void setup(csapex::NodeModifier& node_modifier) = 0;

    /**
     * @brief finishSetup is called after the node is completely set up.
     *
     * By default, this does nothing. The method can be used to make adjustments
     * to the node after it has been fully initialized, e.g. after deserialization.
     */
    virtual void finishSetup();

    /**
     * @brief tearDown is used after the last call to process, before
     * the node is completely destroyed.
     *
     * The method can be used to free any ressources the node is holding
     * that have to released before the destructor is called.
     */
    virtual void tearDown();

    /**
     * @brief canProcess is used to signal, whether the Node::process can be called.
     *
     * By default, this function returns <b>true</b>. It can be reimplemented to
     * disable processing based on internal state.
     *
     * @return <b>true</b>, iff the node can process data.
     *
     * @see Node::yield
     */
    virtual bool canProcess() const;

    /**
     * @brief process is the main function of a node and performes the node's calculation.
     *
     * The method reads from inputs and publishes on outputs.
     * Sources (nodes w/o inputs) and Sinks (nodes w/o outputs) are also allowed.
     *
     * @param node_modifier The modifier to change this node
     * @param parameters All parameters of this node
     * @param continuation The continuation to return the control flow with.
     *
     * @warning <em>This overload is only called, if Node::isAsynchronous returns <b>true</b></em>.
     *
     * @see Node::isAsynchronous, Continuation, NodeModifier, Parameterizable
     */
    virtual void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters, Continuation continuation);

    /**
     * @brief process is the main function of a node and performes the node's calculation.
     *
     * The method reads from inputs and publishes on outputs.
     * Sources (nodes w/o inputs) and Sinks (nodes w/o outputs) are also allowed.
     *
     * @param node_modifier The modifier to change this node
     * @param parameters All parameters of this node
     *
     * @note By default, this overload calls Node::process()
     * @warning <em>This overload is only called, if Node::isAsynchronous returns <b>false</b></em>.
     *
     * @see Node::isAsynchronous
     */
    virtual void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters);

    /**
     * @brief process is the main function of a node and performes the node's calculation.
     *
     * The method reads from inputs and publishes on outputs.
     * Sources (nodes w/o inputs) and Sinks (nodes w/o outputs) are also allowed.
     *
     * @note This overload can be used, if no NodeModifier or Parameterizable is needed.
     * @warning <em>This overload is only called, if Node::isAsynchronous returns <b>false</b></em>.
     *
     * @see Node::isAsynchronous
     */
    virtual void process();

    /**
     * @brief processMessageMarkers specifies, if Node::processMarker should be called for incoming
     * messages of type connection_types::MarkerMessage.
     * By default, the method returns false.
     *
     * @return <b>true</b>, iff Node::processMarker should be called.
     *
     * @see Node::processMarker
     */
    virtual bool processNothingMarkers() const;

    /**
     * @brief processMarker is called, whenever a connection_types::MarkerMessage is received.
     *
     * @param marker
     *
     * @see Node::processMessageMarkers
     */
    virtual void processMarker(const connection_types::MessageConstPtr& marker);

    /**
     * @brief activation is called when the node gets activated by receiving a
     * message with an AcitivityModifier
     */
    virtual void activation();

    /**
     * @brief deactivation is called when the node gets deactivated by sending a
     * message with an AcitivityModifier
     */
    virtual void deactivation();

    /**
     * @brief reset is called when the whole graph is reset.
     *
     * The method is expected to reset internal state to be equal to a newly created node.
     */
    virtual void reset();

    /**
     * @brief isAsynchronous specifies whether this node is processing asynchronously.
     *
     * Processing an asynchronous node is expected to take a longer time.
     * While the node is processing, other nodes should be scheduled as well.
     * The node is expected to return the control flow by calling the continuation that is
     * given to Node::process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters, Continuation continuation).
     * By default, the method returns false.
     *
     * @return <b>true</b>, iff the node processes asynchronously.
     *
     * @see  Node::process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters, Continuation continuation).
     */
    virtual bool isAsynchronous() const;

    /**
     * @brief isIsolated specifies whether this node is node participating in the calculation graph.
     *
     * This is rarely necessary, but can be used for nodes without any I/O ports.
     * By default, the method returns false.
     *
     * @return <b>true</b>, iff the node is isolated.
     */
    virtual bool isIsolated() const;

    /**
     * @brief canRunInSeparateProcess specifies whether this node can be executed in a sub process
     *
     * This means that the node cannot access global variables.
     *
     * @return <b>true</b>, iff the node can run in a sub process.
     */
    virtual bool canRunInSeparateProcess() const;

    /**
     * @brief stateChanged is an event function that is called, when the NodeState has changed.
     */
    virtual void stateChanged();

    /**
     * @brief getProperties is used to probe the node for high level properties.
     *
     * This can be used to specifiy properties by which nodes can be searched in the UI.
     *
     * @param[out] properties A list of properties to be modified.
     */
    virtual void getProperties(std::vector<std::string>& properties) const;

protected:
    /**
     * @brief yield is used to notify the system, that process may be called.
     *
     * This is mainly necessary for source nodes that produce data concurrently.
     *
     * @warning If this method is not called by source nodes, the processing function
     * might never be called!
     */
    void yield() const;

public:
    mutable StreamRelay adebug;  ///< Debug output stream
    mutable StreamRelay ainfo;   ///< Log output stream
    mutable StreamRelay awarn;   ///< Warning output stream
    mutable StreamRelay aerr;    ///< Error output stream

protected:
    csapex::NodeModifier* node_modifier_;
    csapex::Parameterizable* parameters_;
    csapex::NodeHandlePtr node_handle_;

    long guard_;  ///< Memory corruption indicator
};

}  // namespace csapex

#endif  // NODE_H_
