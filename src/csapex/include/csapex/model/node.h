#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/tag.h>
#include <csapex/model/memento.h>
#include <csapex/model/error_state.h>
#include <csapex/model/box.h>

/// SYSTEM
#include <QObject>
#include <QIcon>

namespace csapex {

class Node : public QObject, public ErrorState
{
    Q_OBJECT

    friend class Box;
    friend class GraphIO;
    friend class Graph;

    friend class command::AddConnector;

public:
    typedef boost::shared_ptr<Node> Ptr;

public:
    Node();
    virtual ~Node();

    void setType(const std::string& type);
    std::string getType();

    void setCategory(const std::string& category) __attribute__ ((deprecated));

    void addTag(const Tag& tag);
    std::vector<Tag> getTags() const;

    void setIcon(QIcon icon);
    QIcon getIcon();


    virtual bool canBeDisabled() const;
    bool isEnabled();

    virtual void setState(Memento::Ptr memento);
    virtual Memento::Ptr getState() const;

    /// TODO: get rid of this
    virtual void setBox(Box* box);
    Box* getBox() const;

    template <typename T>
    ConnectorIn* addInput(const std::string& label, bool optional = false) {
        return addInput(T::make(), label, optional);
    }

    template <typename T>
    ConnectorOut* addOutput(const std::string& label) {
        return addOutput(T::make(), label);
    }

    ConnectorIn* addInput(ConnectionTypePtr type, const std::string& label, bool optional);
    ConnectorOut* addOutput(ConnectionTypePtr type, const std::string& label);

    void addInput(ConnectorIn* in) __attribute__ ((deprecated));
    void addOutput(ConnectorOut* out) __attribute__ ((deprecated));

    int countInputs();
    int countOutputs();

    ConnectorIn* getInput(const unsigned int index);
    ConnectorOut *getOutput(const unsigned int index);

    ConnectorIn* getInput(const std::string& uuid);
    ConnectorOut* getOutput(const std::string& uuid);

    void removeInput(ConnectorIn *in);
    void removeOutput(ConnectorOut *out);

    void setSynchronizedInputs(bool sync);

private:
    void errorEvent(bool error, const std::string &msg, ErrorLevel level);

    void registerInput(ConnectorIn* in);
    void registerOutput(ConnectorOut* out);

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source);
    virtual void allConnectorsArrived();

    virtual void enable(bool e);
    virtual void enable();
    virtual void disable(bool e);
    virtual void disable();
    virtual void connectorChanged();

    virtual void tick();

    virtual void updateModel();

Q_SIGNALS:
    void modelChanged();
    void toggled(bool);
    void started();

protected:
    std::string type_;
    mutable std::vector<Tag> tags_;
    QIcon icon_;
    bool enabled_;

private:
    Box* box_;

    std::vector<ConnectorIn*> input;

    std::vector<ConnectorOut*> output;
};

}

#endif // NODE_H_
