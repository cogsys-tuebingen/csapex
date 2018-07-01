#ifndef TEST_EXCEPTION_HANDLER_H
#define TEST_EXCEPTION_HANDLER_H

#include <csapex/core/exception_handler.h>
#include <csapex/core/csapex_core.h>
#include <csapex/utility/error_handling.h>

namespace csapex
{
class TestExceptionHandler : public ExceptionHandler
{
public:
    TestExceptionHandler() : ExceptionHandler(false)
    {
    }

    virtual void handleAssertionFailure(const csapex::Failure& assertion)
    {
        assertion.printStackTrace();
        csapex::error_handling::stop_request()();
    }
};

}  // namespace csapex

#endif  // TEST_EXCEPTION_HANDLER_H
