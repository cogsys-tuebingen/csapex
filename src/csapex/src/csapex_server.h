#ifndef CSAPEX_H
#define CSAPEX_H

/// PROJECT
#include <csapex/core/csapex_core.h>
#include <csapex/core/settings.h>
#include <csapex/command/command_fwd.h>
#include <csapex/utility/exceptions.h>
#include <csapex/core/exception_handler.h>
#include <csapex/model/observer.h>

/// SYSTEM
#include <memory>
#include <condition_variable>

namespace csapex
{
struct CsApexServer : public Observer
{
public:
    CsApexServer(Settings& settings, ExceptionHandler& handler);
    ~CsApexServer();

    int run();

private:
    Settings& settings;

    ExceptionHandler& handler;

    CsApexCorePtr core;
};

}  // namespace csapex

#endif  // CSAPEX_H
