/// HEADER
#include <csapex_server.h>

/// PROJECT
#include <csapex/core/csapex_core.h>
#include <csapex/core/settings/settings_local.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/error_handling.h>
#include <csapex/utility/exceptions.h>
#include <csapex/utility/thread.h>
#include <csapex/io/server.h>

/// SYSTEM
#include <iostream>
#include <thread>
#include <boost/program_options.hpp>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/version.hpp>

#if (BOOST_VERSION / 100000) >= 1 && (BOOST_VERSION / 100 % 1000) >= 54
namespace bf3 = boost::filesystem;
#else
namespace bf3 = boost::filesystem3;
#endif

namespace po = boost::program_options;

using namespace csapex;


CsApexServer::CsApexServer(Settings& settings, ExceptionHandler& handler)
    : settings(settings), handler(handler)
{
    csapex::thread::set_name("cs::APEX main");
}

CsApexServer::~CsApexServer()
{
}

int CsApexServer::run()
{
    core = std::make_shared<CsApexCore>(settings, handler);

    Server server(core, false);

    GraphFacadePtr root = core->getRoot();
    csapex::error_handling::stop_request().connect([&server](){
        server.stop();
    });
    csapex::error_handling::init();

    core->startup();

    server.start();

    return 0;
}

int main(int argc, char** argv)
{
    SettingsLocal settings;

    int effective_argc = argc;
    std::string path_to_bin(argv[0]);

    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "show help message")
            ("debug", "enable debug output")
            ("dump", "show variables")
            ("paused", "start paused")
            ("headless", "run without gui")
            ("threadless", "run without threading")
            ("fatal_exceptions", "abort execution on exception")
            ("disable_thread_grouping", "by default create one thread per node")
            ("input", "config file to load")
            ;

    po::positional_options_description p;
    p.add("input", 1);

    // first check for --headless or --fatal_exceptions parameter
    // this has to be done before the qapp can be created, which
    // has to be done before parameters can be read.
    bool headless = false;
    bool fatal_exceptions = false;
    for(int i = 1; i < effective_argc; ++i) {
        std::string arg(argv[i]);
        if(arg == "--headless") {
            headless = true;
        } else if(arg == "--fatal_exceptions") {
            fatal_exceptions = true;
        }
    }

    if(!headless) {
        // if headless not requested, check if there is a display
        // if not, we enforce headless mode
#if WIN32
        if (false) {
#else
        if (!getenv("DISPLAY")) {
#endif
            headless = true;
            std::cout << "warning: enforcing headless mode because there is no display detected" << std::endl;
        }
    }


    std::shared_ptr<ExceptionHandler> handler(new ExceptionHandler(fatal_exceptions));

    // filters ros remappings
    std::vector<std::string> remapping_args;
    std::vector<std::string> rest_args;
    for(int i = 1; i < effective_argc; ++i) {
        std::string arg(argv[i]);
        if(arg.find(":=") != std::string::npos)  {
            remapping_args.push_back(arg);
        } else {
            rest_args.push_back(arg);
        }
    }

    // now check for remaining parameters
    po::variables_map vm;
    std::vector<std::string> additional_args;

    try {
        po::parsed_options parsed = po::command_line_parser(rest_args).options(desc).positional(p).run();

        po::store(parsed, vm);

        po::notify(vm);

        additional_args = po::collect_unrecognized(parsed.options, po::include_positional);

    } catch(const std::exception& e) {
        std::cerr << "cannot parse parameters: " << e.what() << std::endl;
        return 4;
    }

    // add ros remappings
    additional_args.insert(additional_args.end(), remapping_args.begin(), remapping_args.end());


    // display help?
    if(vm.count("help")) {
        std::cerr << desc << std::endl;
        return 1;
    }
    if(vm.count("dump")) {
        std::cout << "to be passed on:\n";
        for(std::size_t i = 0; i < additional_args.size(); ++i) {
            std::cout << additional_args[i] << '\n';
        }
        std::cout << std::flush;
        return 0;
    }

    // which file to use?
    if (vm.count("input")) {
        settings.set("config",vm["input"].as<std::string>());
    } else {
        settings.set("config",Settings::default_config);
    }

    if(!settings.knows("path_to_bin")) {
        settings.addTemporary(csapex::param::ParameterFactory::declareFileInputPath("path_to_bin", path_to_bin));
    } else {
        settings.set("path_to_bin", path_to_bin);
    }

    settings.set("debug", vm.count("debug") > 0);
    settings.set("headless", headless);
    settings.set("threadless", vm.count("threadless") > 0);
    settings.set("thread_grouping", vm.count("disable_thread_grouping") == 0);
    settings.set("additional_args", additional_args);
    settings.set("initially_paused", vm.count("paused") > 0);

    settings.set("access-test", std::string("access granted."));

    // start the app
    CsApexServer m(settings, *handler);
    try {
        return m.run();

    } catch(const csapex::Failure& af) {
        std::cerr << af.what() << std::endl;
        return 42;
    }
}
