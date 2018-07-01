/// HEADER
#include <csapex_testing/csapex_test_case.h>

/// PROJECT
#include <csapex/utility/error_handling.h>

using namespace csapex;

CsApexTestCase::CsApexTestCase()
{
    csapex::error_handling::init();
}

CsApexTestCase::~CsApexTestCase()
{
}
