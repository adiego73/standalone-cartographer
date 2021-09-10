#include "cartographer_mapping.hpp"
using namespace adiego73;

void server_run(std::unique_ptr<CartographerMapping>& p_mapping);
int
main()
{
    std::unique_ptr<CartographerMapping> mapping = std::make_unique<CartographerMapping>();
    mapping->configure("config/cartographer", "my_config.lua");

    std::thread t_mapping([&]() {
        mapping->run();
    });

    std::thread t_server([&]() {
        server_run(mapping);
    });

    t_mapping.join();
    t_server.join();

    std::cout << "DONE! See you soon";

    return 0;
}

void
server_run(std::unique_ptr<CartographerMapping>& p_mapping)
{
    std::string map_name;
    char option;
    do {
        std::cout << "What should I do? \n";
        std::cout << "\t1. Press 'q' to quit \n";
        std::cout << "\t2. Press 'c' to run final optimization and store the map \n";
        std::cout << "\t3. Press 's' to store current state into a pbstream \n";
        std::cout << "\t4. Press 'l' to load a previously stored current state \n";
        std::cout << "Choose your option (q/c/s/l): ";
        std::cin >> option;
        std::cout << std::endl;

        if (option == 's') {
            char include_submaps = 'y';
            std::cout << "Should I include incomplete submaps? (n/Y): ";
            std::cin >> include_submaps;
            std::cout << "\nState file name? ";
            std::cin >> map_name;

            p_mapping->storeCurrentState((include_submaps == 'y' || include_submaps == 'Y'), map_name);

        } else if (option == 'l') {
            std::string state_stream_path;
            std::cout << "Which is the path of the stored state file? ";
            std::cin >> state_stream_path;

            if (p_mapping->loadStoredState(state_stream_path)) {
                std::cout << "State file " << state_stream_path << " loaded." << std::endl;
            }

        } else if (option == 'c') {
            p_mapping->doFinalOptimization();

            std::cout << "\nMap name? ";
            std::cin >> map_name;

            std::cout << "Building the map....\n";

            p_mapping->createMap(map_name);
        }

    } while (option != 'q');


    p_mapping->stop();
}
