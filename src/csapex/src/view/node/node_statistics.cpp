/// HEADER
#include <csapex/view/node/node_statistics.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/node_facade.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/generic_state.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

using namespace csapex;

NodeStatistics::NodeStatistics(NodeFacade *node)
    : node_facade_(node)
{

}

QTreeWidgetItem * NodeStatistics::createDebugInformationConnector(const ConnectorDescription& connector) const
{
    QTreeWidgetItem* connector_widget = new QTreeWidgetItem;
    connector_widget->setText(0, QString::fromStdString(connector.id.getShortName()));
    connector_widget->setIcon(0, QIcon(":/connector.png"));

    QTreeWidgetItem* uuid = new QTreeWidgetItem;
    uuid->setText(0, "UUID");
    uuid->setText(1, QString::fromStdString(connector.id.getFullName()));
    connector_widget->addChild(uuid);

    QTreeWidgetItem* label = new QTreeWidgetItem;
    label->setText(0, "Label");
    label->setText(1, QString::fromStdString(connector.label));
    connector_widget->addChild(label);


    QTreeWidgetItem* type = new QTreeWidgetItem;
    type->setText(0, "Type");
    type->setText(1, QString::fromStdString(connector.token_type->descriptiveName()));
    connector_widget->addChild(type);

    return connector_widget;
}

QTreeWidgetItem* NodeStatistics::createDebugInformation(NodeFactory* node_factory) const
{
    QTreeWidgetItem* tl = new QTreeWidgetItem;
    tl->setText(0, QString::fromStdString(node_facade_->getUUID().getFullName()));

    NodeConstructor::Ptr constructor = node_factory->getConstructor(node_facade_->getType());

    tl->setIcon(0, QIcon(QString::fromStdString(constructor->getIcon())));

    {
        QTreeWidgetItem* connectors = new QTreeWidgetItem;
        connectors->setText(0, "Inputs");

        for(const ConnectorDescription& input : node_facade_->getInputs()) {
            QTreeWidgetItem* connector_widget = createDebugInformationConnector(input);

            QTreeWidgetItem* input_widget = new QTreeWidgetItem;
            input_widget->setText(0, "Input");

            QTreeWidgetItem* targets = new QTreeWidgetItem;
            for(const AUUID& output: input.targets) {
                QTreeWidgetItem* target_widget = new QTreeWidgetItem;
                target_widget->setText(0, QString::fromStdString(output.getFullName()));
                target_widget->setIcon(1, QIcon(":/connector.png"));
                targets->addChild(target_widget);
            }
            input_widget->addChild(targets);

            connector_widget->addChild(input_widget);

            connectors->addChild(connector_widget);
        }
        tl->addChild(connectors);
    }
    {
        QTreeWidgetItem* connectors = new QTreeWidgetItem;
        connectors->setText(0, "Outputs");

        for(const ConnectorDescription& output : node_facade_->getNodeHandle()->getExternalOutputDescriptions()) {
            QTreeWidgetItem* output_widget = createDebugInformationConnector(output);

            QTreeWidgetItem* targets = new QTreeWidgetItem;
            targets->setText(0, "Target");
            for(const AUUID& input : output.targets) {
                QTreeWidgetItem* target_widget = new QTreeWidgetItem;
                target_widget->setText(0, QString::fromStdString(input.getFullName()));
                target_widget->setIcon(1, QIcon(":/connector.png"));
                targets->addChild(target_widget);
            }
            output_widget->addChild(targets);

            connectors->addChild(output_widget);
        }
        tl->addChild(connectors);
    }
    {
        QTreeWidgetItem* parameters = new QTreeWidgetItem;
        parameters->setText(0, "Parameters");
        GenericStateConstPtr state = node_facade_->getParameterState();
        for(std::map<std::string, csapex::param::Parameter::Ptr>::const_iterator it = state->params.begin(), end = state->params.end(); it != end; ++it ) {
            csapex::param::Parameter* p = it->second.get();

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
            YAML::Node n;
            p->serialize_yaml(n);
            e << n;

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
        aout_w_txt->setText(0, QString::fromStdString(node_facade_->getLoggerOutput(ErrorState::ErrorLevel::NONE)));
        aout_w->addChild(aout_w_txt);
        streams->addChild(aout_w);

        QTreeWidgetItem* awarn_w = new QTreeWidgetItem;
        awarn_w->setText(0, "warning");
        QTreeWidgetItem* awarn_w_txt = new QTreeWidgetItem;
        awarn_w_txt->setText(0, QString::fromStdString(node_facade_->getLoggerOutput(ErrorState::ErrorLevel::WARNING)));
        awarn_w->addChild(awarn_w_txt);
        streams->addChild(awarn_w);

        QTreeWidgetItem* aerr_w = new QTreeWidgetItem;
        aerr_w->setText(0, "error");
        QTreeWidgetItem* aerr_w_txt = new QTreeWidgetItem;
        aerr_w_txt->setText(0, QString::fromStdString(node_facade_->getLoggerOutput(ErrorState::ErrorLevel::ERROR)));
        aerr_w->addChild(aerr_w_txt);
        streams->addChild(aerr_w);

        tl->addChild(streams);
    }

    return tl;
}
