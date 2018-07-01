#include <csapex_testing/csapex_test_case.h>
#include <csapex/utility/exceptions.h>

void stacktraceTerminate()
{
    std::cerr << "csapex unit tests terminated by call to std::terminate" << std::endl;

    auto ptr = std::current_exception();
    try {
        std::rethrow_exception(ptr);
    } catch (const csapex::Failure& f) {
        std::cerr << f.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    } catch (...) {
        std::cerr << "unknown exception" << std::endl;
    }

    std::quick_exit(123);
}

int main(int argc, char** argv)
{
    std::set_terminate(stacktraceTerminate);

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
