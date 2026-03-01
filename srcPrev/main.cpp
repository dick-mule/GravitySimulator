#include "VulkanApp.hpp"
#include <iostream>

int main() {
    try
    {
        std::print("MAKING APP\n");
        VulkanApp app;
        std::print("RUN APP\n");
        app.run();
        std::print("APP RAN\n");
    } catch ( const std::exception& e )
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }

    return 0;
}