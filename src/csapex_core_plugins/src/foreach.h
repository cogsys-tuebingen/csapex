#ifndef FOREACH_H
#define FOREACH_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_core_plugins/vector_message.h>

namespace csapex {

class Foreach : public csapex::BoxedObject
{
    Q_OBJECT

public:
    Foreach();
    ~Foreach();

    virtual void process();
    virtual void fill(QBoxLayout* layout);

    virtual Connectable* getConnector(const UUID &uuid) const;

    virtual std::vector<ConnectorIn*> getInputs() const;
    virtual std::vector<ConnectorOut*> getOutputs() const;

    virtual ConnectorIn* getInput(const UUID& uuid) const;
    virtual ConnectorOut* getOutput(const UUID& uuid) const;

    virtual void stop();

private Q_SLOTS:
    void appendMessageFrom(Connectable*);

private:
    ConnectorIn* input_;
    ConnectorOut* output_;

    ConnectorIn* in_sub;
    ConnectorOut* out_sub;

    connection_types::VectorMessage::Ptr current_result_;

    int messages_;
    int message_;
};

}

#endif // FOREACH_H
