// main.cpp
#include "limxsdk/ability/ability_manager.h"
#include <iostream>
#include <signal.h>

// Global ability manager instance
limxsdk::ability::AbilityManager* g_abilityManager = nullptr;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received." << std::endl;
    
    if (g_abilityManager) {
        std::cout << "Stopping all abilities..." << std::endl;
        g_abilityManager->stopRemoteServer();
    }
    
    // Terminate program
    exit(signum);
}

int main(int argc, char** argv) {
    std::cout << "LIMX SDK Ability Manager" << std::endl;
    std::cout << "------------------------" << std::endl;
    
    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Load configuration
    std::string configPath = "config.yaml";
    if (argc > 1) {
        configPath = argv[1];
    }
    
    std::cout << "Loading configuration from: " << configPath << std::endl;

    // Create ability manager with CLI port 8888
    limxsdk::ability::AbilityManager abilityManager(configPath);
    g_abilityManager = &abilityManager;
    
    // Start remote CLI server
    if (!abilityManager.startRemoteServer()) {
        std::cerr << "Failed to start remote CLI server. Exiting..." << std::endl;
        return 1;
    }
    
    // Print usage instructions
    std::cout << "\nConnect to the CLI server using Telnet or Netcat:" << std::endl;
    std::cout << "  $ telnet localhost 8888" << std::endl;
    std::cout << "Type 'help' for available commands." << std::endl;
    std::cout << "Press Ctrl+C to exit." << std::endl;
    
    // Keep main thread alive
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}
