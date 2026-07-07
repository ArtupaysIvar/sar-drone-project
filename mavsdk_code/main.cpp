#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace mavsdk;
using namespace std::chrono_literals;

int main()
{
    Mavsdk mavsdk(
        Mavsdk::Configuration(ComponentType::GroundStation));

    auto connection_result =
        mavsdk.add_any_connection("udpin://0.0.0.0:14540");

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: "
                  << connection_result << std::endl;
        return 1;
    }

    std::cout << "Waiting for PX4..." << std::endl;

    while (mavsdk.systems().empty()) {
        std::this_thread::sleep_for(1s);
    }

    auto system = mavsdk.systems().at(0);

    while (!system->is_connected()) {
        std::this_thread::sleep_for(1s);
    }

    std::cout << "Vehicle discovered." << std::endl;

    Telemetry telemetry(system);
    Action action(system);

    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for vehicle health..." << std::endl;
        std::this_thread::sleep_for(1s);
    }

    std::cout << "Setting takeoff altitude to 10 m" << std::endl;

    auto result = action.set_takeoff_altitude(10.0f);
    if (result != Action::Result::Success) {
        std::cerr << "Failed to set takeoff altitude: "
                  << result << std::endl;
        return 1;
    }

    std::cout << "Arming..." << std::endl;

    result = action.arm();
    if (result != Action::Result::Success) {
        std::cerr << "Arm failed: "
                  << result << std::endl;
        return 1;
    }

    std::cout << "Taking off..." << std::endl;

    result = action.takeoff();
    if (result != Action::Result::Success) {
        std::cerr << "Takeoff failed: "
                  << result << std::endl;
        return 1;
    }

    std::cout << "Waiting 20 seconds..." << std::endl;
    std::this_thread::sleep_for(20s);

    std::cout << "Landing..." << std::endl;

    result = action.land();
    if (result != Action::Result::Success) {
        std::cerr << "Land failed: "
                  << result << std::endl;
        return 1;
    }

    std::cout << "Done." << std::endl;

    return 0;
}