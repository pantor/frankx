#define CATCH_CONFIG_MAIN
#include <random>

#include <catch2/catch.hpp>
#include <Eigen/Core>

#include <movex/otg/parameter.hpp>
#include <movex/otg/quintic.hpp>


using namespace movex;


TEST_CASE("Quintic") {
    InputParameter<3> input;
    input.current_position = {0.0, 0.0, 0.0};
    input.current_velocity = {0.0, 0.0, 0.0};
    input.current_acceleration = {0.0, 0.0, 0.0};
    input.target_position = {1.0, 1.0, 1.0};
    input.target_velocity = {0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0};

    input.max_velocity = {1.0, 1.0, 1.0};
    input.max_acceleration = {1.0, 1.0, 1.0};
    input.max_jerk = {1.0, 1.0, 1.0};

    OutputParameter<3> output;

    Quintic<3> otg {0.005};

    std::vector<OutputParameter<3>> trajectory(500);

    while (otg.update(input, output) == Result::Working) {
        trajectory.push_back(output);

        input.current_position = output.new_position;
        input.current_velocity = output.new_velocity;
        input.current_acceleration = output.new_acceleration;
    }

    REQUIRE( trajectory.size() == 4.415 / 0.005 );
}