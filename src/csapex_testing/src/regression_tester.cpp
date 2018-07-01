#include <csapex/plugin/plugin_locator.h>
#include <csapex/core/settings/settings_impl.h>
#include <csapex/core/csapex_core.h>
#include <csapex/core/exception_handler.h>
#include <csapex/utility/subprocess.h>

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/version.hpp>

#if (BOOST_VERSION / 100000) >= 1 && (BOOST_VERSION / 100 % 1000) >= 54
namespace bf3 = boost::filesystem;
#else
namespace bf3 = boost::filesystem3;
#endif

using namespace csapex;

bool run_tests(CsApexCore& core, const std::vector<bf3::path>& reg_test_files)
{
    bool error_happened = false;

    for (const bf3::path& reg_test : reg_test_files) {
        std::string filename = reg_test.filename().string();
        ;
        int expected_return_code = 0;

        // The files must be named <return code>_<file name>.apex
        std::size_t split = filename.find_first_of("_");
        if (split == std::string::npos) {
            std::cout << "Warning: Do not know the expected return code of test file '" << filename << "'\n";
            std::cout << "         The file does not follow the convention <return code>_<file name>.apex" << '\n';
            std::cout << "         Assuming return code 0" << std::endl;
        } else {
            std::stringstream conv(filename.substr(0, split));
            conv >> expected_return_code;
            filename = filename.substr(split + 1);
        }

        std::cout << "[ RUN APEX REGRESSION TEST ] " << filename << std::endl;

        {
            Subprocess sp(reg_test.filename().string());
            sp.fork([&]() {
                chdir(reg_test.parent_path().string().c_str());

                core.load(reg_test.string());

                std::mutex running_mutex;
                std::condition_variable shutdown;
                auto connection = core.shutdown_requested.connect([&]() { shutdown.notify_all(); });

                core.startMainLoop();
                auto lock = std::unique_lock<std::mutex>(running_mutex);
                shutdown.wait_for(lock, std::chrono::seconds(5));

                if (core.isMainLoopRunning()) {
                    std::cerr << "Test timed out!" << std::endl;
                    core.abort();
                }

                core.joinMainLoop();

                return core.getReturnCode();
            });

            int return_code = sp.join();
            if (return_code == expected_return_code) {
                std::cout << "[                       OK ] " << std::endl;

            } else {
                error_happened = true;

                std::cout << "[                   FAILED ] Error code: " << return_code << ", exepected: " << expected_return_code << std::endl;

                std::string out = sp.getChildStdOut();
                if (!out.empty()) {
                    std::cout << "Ouput was: \n";
                    std::cout << out << std::endl;
                }

                std::string err = sp.getChildStdErr();
                if (!err.empty()) {
                    std::cout << "Error Ouput was: \n";
                    std::cout << err << std::endl;
                }
            }
        }
    }

    return error_happened;
}

int main(int argc, char* argv[])
{
    ExceptionHandler eh(false);
    SettingsImplementation settings;

    std::string path_to_bin(argv[0]);
    settings.set("path_to_bin", path_to_bin);
    settings.set("require_boot_plugin", false);

    CsApexCore core(settings, eh);
    PluginLocatorPtr locator = core.getPluginLocator();

    std::vector<bf3::path> reg_test_files;
    auto test_dirs = locator->getPluginPaths("regression_tests");
    if (test_dirs.empty()) {
        std::cout << "No regression tests found" << std::endl;
        return 0;
    }

    for (const std::string& dir_string : test_dirs) {
        boost::filesystem::path directory(dir_string);

        boost::filesystem::directory_iterator dir(directory);
        boost::filesystem::directory_iterator end;

        for (; dir != end; ++dir) {
            boost::filesystem::path path = dir->path();
            reg_test_files.push_back(path);
        }
    }

    return run_tests(core, reg_test_files) ? 1 : 0;
}
