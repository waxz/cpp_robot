//
// Created by waxz on 4/17/24.
//





#include "lyra/lyra.hpp"


#include "common/signals.h"
#include "common/functions.h"
#include "common/suspend.h"
#include "common/task.h"


#include "dds_handler.h"
#include "dds_handler_variant.h"

//#include "dds_handler_demo.h"

using namespace eprosima::fastdds::dds;
#define STATIC_MEMORY_SIZE 100000000
static unsigned char memory_pool[STATIC_MEMORY_SIZE];
static const ta_cfg_t memory_pool_cfg = {
        memory_pool,
        &memory_pool[sizeof(memory_pool)],
        512,
        16,
        8,
};


int main(int argc, char** argv){



    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
    set_signal_handler(my_handler);

    bool get_help = false;
    std::string exe_name;
    std::string name;

    std::string xml_file;
    std::string profile;



    // get topic xml
    auto cli
            = lyra::exe_name(exe_name)
              | lyra::help(get_help)
              | lyra::opt(xml_file, "xml_file")["-x"]["--xml_file"]("xml_file")
              | lyra::opt(profile, "profile")["-p"]["--profile"]("profile")
                | lyra::opt(name, "name")["-n"]["--name"]("name")


    ;

    auto result = cli.parse({argc, argv});
    if (get_help) {
        std::cout << cli << std::endl;
        return 0;

    }


    if (!result) {
        std::cerr << "Error in command line: " << result.message() << std::endl;
        return 0;
    }



    if (xml_file.empty()
    || name.empty()
    || profile.empty()){
        std::cout << cli << std::endl;
    }

    // create participant with profile


#if 0
    const char* toml_file = "dds_config.toml";
    dds_helper::DdsHandlerVariant handlerVariant;
    handlerVariant.create(toml_file,&memory_pool_cfg);
#endif
#if 0
    DdsSimpleHandler handler2;
    bool ok2 = handler2.init( xml_file.c_str(), profile.c_str());

    if(!ok2){
        std::cout << "create dds handler failed, exit"<< std::endl;
        return 0;
    }
#endif


    ParticipantConfig config;
    config.xml = xml_file;
    config.profile = profile;
    config.name = name;
    dds_helper::DdsSimpleParticipant handler;


    int ok = handler.init( config);

    if(ok < 0){
        std::cout << "create dds handler failed, exit"<< std::endl;
        return 0;
    }


    common::LoopHelper loopHelper;
    loopHelper.set_fps(10.0);
    while (program_run){
        loopHelper.start();

        loopHelper.sleep();

    }



}