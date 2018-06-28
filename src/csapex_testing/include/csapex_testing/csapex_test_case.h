#ifndef CSAPEX_TEST_CASE_H
#define CSAPEX_TEST_CASE_H

/// SYSTEM
#include "gtest/gtest.h"

namespace csapex
{

class CsApexTestCase : public ::testing::Test {
protected:
    CsApexTestCase();

    virtual ~CsApexTestCase();
};
}

#endif // CSAPEX_TEST_CASE_H
