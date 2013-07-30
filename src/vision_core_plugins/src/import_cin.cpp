/// HEADER
#include "import_cin.h"

/// PROJECT
#include <csapex/box.h>
#include <csapex/connector_out.h>
#include <csapex/connection_type_manager.h>
#include <csapex/stream_interceptor.h>
#include <vision_evaluator/messages_default.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <yaml-cpp/eventhandler.h>

PLUGINLIB_EXPORT_CLASS(csapex::ImportCin, csapex::BoxedObject)

using namespace csapex;

ImportCin::ImportCin()
    : connector_(NULL)
{
    setCategory("ConsoleIO");
    setIcon(QIcon(":/terminal.png"));
}

void ImportCin::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        connector_ = new ConnectorOut(box_, 0);
        connector_->setLabel("Anything");
        connector_->setType(connection_types::AnyMessage::make());

        box_->addOutput(connector_);
    }
}

void ImportCin::tick()
{
    std::string in;
    in = StreamInterceptor::instance().getCin();
    buffer << in;

    std::string unused = buffer.str();

    int pos = unused.find_first_of("---");
    if(pos == -1 || unused.empty()) {
        return;
    }

    std::stringstream docstream;

    int iter = 0;
    while(pos != -1) {
        std::string sub = unused.substr(0, pos);

        unused = unused.substr(pos+3);

        if(pos != -1) {
            if(!docstream.str().empty()) {
                docstream << "\n---\n";
            }
            docstream << sub;
        }

        pos = unused.find_first_of("---");

        if(iter++ >= 10) {
            std::cout << "break out!" << std::endl;
            break;
        }
    }

    buffer.str(std::string());
    buffer << unused;

    try {
        YAML::Node doc;

        YAML::Parser parser(docstream);
        while(parser.GetNextDocument(doc)) {
            std::string type;
            doc["type"] >> type;

            ConnectionType::Ptr msg = ConnectionTypeManager::createMessage(type);
            if(!msg) {
                std::cout << "could not deserialize message: \n";
                YAML::Emitter e;
                e << doc;
                std::cout << e.c_str() << std::endl;
                continue;
            }

            msg->readYaml(doc);

            connector_->setType(msg->clone());
            connector_->publish(msg);
        }
    } catch(YAML::ParserException& e) {
        std::cout << "YAML::ParserException: " << e.what() << "\n";
    }

}

void ImportCin::messageArrived(ConnectorIn *source)
{
    // NO INPUT
}
