/// HEADER
#include <csapex/utility/assert.h>

/// SYSTEM
#include <assert.h>
#include <stdexcept>
#include <sstream>
#include <iostream>

void apex_assert(bool assertion)
{
    apex_assert_hard(assertion);
}

void apex_assert_hard(bool assertion)
{
    if(!assertion) {
        std::stringstream ss;
        ss << "assertion failed in " << __FILE__ << ", line " << __LINE__ ;
        throw std::runtime_error(ss.str());
    }
}

void apex_assert_soft(bool assertion)
{
    if(!assertion) {
        std::cerr << "[cs::APEX - SOFT ASSERTION FAILED] file " << __FILE__ << ", line " << __LINE__  << std::endl;
    }
}
