/// HEADER
#include <csapex/io/server.h>

/// COMPONENT
#include <csapex/core/csapex_core.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/notification_message.h>
#include <csapex/io/request.h>
#include <csapex/io/response.h>
#include <csapex/io/feedback.h>
#include <csapex/command/update_parameter.h>
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/io/protcol/parameter_changed.h>
#include <csapex/io/protcol/command_broadcasts.h>
#include <csapex/io/protcol/graph_broadcasts.h>
#include <csapex/io/protcol/core_notes.h>
#include <csapex/io/graph_server.h>

/// SYSTEM
#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>

using namespace csapex;
using boost::asio::ip::tcp;

Server::Server(CsApexCore &core)
    : core_(core)
{
    observe(core_.shutdown_requested, [this](){
        stop();
    });
}

Server::~Server()
{
}


