/// HEADER
#include "import_cin.h"

/// PROJECT

#include <csapex/model/connector_out.h>
#include <csapex/manager/connection_type_manager.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/model/message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <yaml-cpp/eventhandler.h>

CSAPEX_REGISTER_CLASS(csapex::ImportCin, csapex::Node)

using namespace csapex;

ImportCin::ImportCin()
    : connector_(NULL)
{
    addTag(Tag::get("ConsoleIO"));
    addTag(Tag::get("General"));
    addTag(Tag::get("Input"));
    setIcon(QIcon(":/terminal.png"));
}

void ImportCin::fill(QBoxLayout *)
{
    if(connector_ == NULL) {
        connector_ = addOutput<connection_types::AnyMessage>("Anything");
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
