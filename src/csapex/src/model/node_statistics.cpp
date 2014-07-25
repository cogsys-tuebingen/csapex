/// HEADER
#include <csapex/model/node_statistics.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/manager/box_manager.h>

using namespace csapex;

NodeStatistics::NodeStatistics(Node *node)
    : node_(node)
{

}

QTreeWidgetItem * NodeStatistics::createDebugInformationConnector(Connectable* connector) const
{
    QTreeWidgetItem* connector_widget = new QTreeWidgetItem;
    connector_widget->setText(0, connector->getUUID().getShortName().c_str());
    connector_widget->setIcon(0, QIcon(":/connector.png"));

    QTreeWidgetItem* uuid = new QTreeWidgetItem;
    uuid->setText(0, "UUID");
    uuid->setText(1, connector->getUUID().getFullName().c_str());
    connector_widget->addChild(uuid);

    QTreeWidgetItem* label = new QTreeWidgetItem;
    label->setText(0, "Label");
    label->setText(1, connector->getLabel().c_str());
    connector_widget->addChild(label);


    QTreeWidgetItem* type = new QTreeWidgetItem;
    type->setText(0, "Type");
    type->setText(1, connector->getType()->name().c_str());
    connector_widget->addChild(type);

    return connector_widget;
}

QTreeWidgetItem* NodeStatistics::createDebugInformation() const
{
    QTreeWidgetItem* tl = new QTreeWidgetItem;
    tl->setText(0, node_->getUUID().c_str());

    NodeConstructor::Ptr constructor = BoxManager::instance().getConstructor(node_->getType());

    tl->setIcon(0, constructor->getIcon());

    {
        QTreeWidgetItem* connectors = new QTreeWidgetItem;
        connectors->setText(0, "Inputs");
        for(int i = 0, n = node_->countInputs(); i < n; ++i) {
            ConnectorIn* connector = node_->getInput(i);

            QTreeWidgetItem* connector_widget = createDebugInformationConnector(connector);

            QTreeWidgetItem* input = new QTreeWidgetItem;
            input->setText(0, "Input");

            QTreeWidgetItem* target_widget = new QTreeWidgetItem;
            if(connector->isConnected()) {
                Connectable* target = connector->getSource();
                target_widget->setText(0, target->getUUID().c_str());
                //target_widget->setIcon(1, target->getNode()->getIcon());
                //target_widget->setText(1, target->getNode()->getType().c_str());
                target_widget->setIcon(1, QIcon(":/connector.png"));
            } else {
                target_widget->setText(0, "not connected");
                target_widget->setIcon(1, QIcon(":/disconnected.png"));
            }

            input->addChild(target_widget);

            connector_widget->addChild(input);

            connectors->addChild(connector_widget);
        }
        tl->addChild(connectors);
    }
    {
        QTreeWidgetItem* connectors = new QTreeWidgetItem;
        connectors->setText(0, "Outputs");
        for(int i = 0, n = node_->countOutputs(); i < n; ++i) {
            ConnectorOut* connector = node_->getOutput(i);

            QTreeWidgetItem* connector_widget = createDebugInformationConnector(connector);

            QTreeWidgetItem* targets = new QTreeWidgetItem;
            targets->setText(0, "Target");
            for(ConnectorOut::TargetIterator it = connector->beginTargets(); it != connector->endTargets(); ++it) {
                ConnectorIn* target = *it;
                QTreeWidgetItem* target_widget = new QTreeWidgetItem;
                target_widget->setText(0, target->getUUID().c_str());
                //target_widget->setIcon(1, target->getNode()->getIcon());
                //target_widget->setText(1, target->getNode()->getType().c_str());
                target_widget->setIcon(1, QIcon(":/connector.png"));
                targets->addChild(target_widget);
            }
            connector_widget->addChild(targets);

            connectors->addChild(connector_widget);
        }
        tl->addChild(connectors);
    }
    {
        QTreeWidgetItem* parameters = new QTreeWidgetItem;
        parameters->setText(0, "Parameters");
        for(std::map<std::string, param::Parameter::Ptr>::const_iterator it = node_->parameter_state_.params.begin(); it != node_->parameter_state_.params.end(); ++it ) {
            param::Parameter* p = it->second.get();

            QTreeWidgetItem* param = new QTreeWidgetItem;
            param->setText(0, p->name().c_str());


            QTreeWidgetItem* params_descr_widget = new QTreeWidgetItem;
            params_descr_widget->setText(0, "Description");
            params_descr_widget->setText(1, p->description().toString().c_str());
            param->addChild(params_descr_widget);

            QTreeWidgetItem* params_type_widget = new QTreeWidgetItem;
            params_type_widget->setText(0, "Value type");
            params_type_widget->setText(1, type2name(p->type()).c_str());
            param->addChild(params_type_widget);

            YAML::Emitter e;
            p->write(e);
            QTreeWidgetItem* params_type_data = new QTreeWidgetItem;
            params_type_data->setText(0, "Data");
            params_type_data->setText(1, e.c_str());
            param->addChild(params_type_data);

            QTreeWidgetItem* enabled = new QTreeWidgetItem;
            enabled->setText(0, "Enabled?");
            enabled->setText(1, p->isEnabled() ? "true" : "false");
            param->addChild(enabled);

            QTreeWidgetItem* interactive = new QTreeWidgetItem;
            interactive->setText(0, "Interactive?");
            interactive->setText(1, p->isInteractive() ? "true" : "false");
            param->addChild(interactive);

            parameters->addChild(param);

            param->setData(0, Qt::UserRole, p->isEnabled());
        }
        tl->addChild(parameters);
    }
    {
        QTreeWidgetItem* streams = new QTreeWidgetItem;
        streams->setText(0, "Output");

        QTreeWidgetItem* aout_w = new QTreeWidgetItem;
        aout_w->setText(0, "output");
        QTreeWidgetItem* aout_w_txt = new QTreeWidgetItem;
        aout_w_txt->setText(0, node_->ainfo.history().str().c_str());
        aout_w->addChild(aout_w_txt);
        streams->addChild(aout_w);

        QTreeWidgetItem* awarn_w = new QTreeWidgetItem;
        awarn_w->setText(0, "warning");
        QTreeWidgetItem* awarn_w_txt = new QTreeWidgetItem;
        awarn_w_txt->setText(0, node_->awarn.history().str().c_str());
        awarn_w->addChild(awarn_w_txt);
        streams->addChild(awarn_w);

        QTreeWidgetItem* aerr_w = new QTreeWidgetItem;
        aerr_w->setText(0, "error");
        QTreeWidgetItem* aerr_w_txt = new QTreeWidgetItem;
        aerr_w_txt->setText(0, node_->aerr.history().str().c_str());
        aerr_w->addChild(aerr_w_txt);
        streams->addChild(aerr_w);

        tl->addChild(streams);
    }

    return tl;
}
