#define CATCH_CONFIG_MAIN
#include <random>

#include <catch2/catch.hpp>

#include <frankx/frankx.hpp>


using namespace frankx;


TEST_CASE("Geometry") {
    SECTION("Robot state: RML Vector") {
        Robot robot("172.16.0.2");
        auto state = robot.readOnce();
        auto pose = franka::CartesianPose(state.O_T_EE_c, state.elbow_c);

        auto vector = Affine(pose).vector_with_elbow(pose.elbow[0]);
        REQUIRE( vector[3] < 0.1 );
        REQUIRE( vector[3] > -0.1 );
        REQUIRE( vector[4] < 0.1 );
        REQUIRE( vector[4] > -0.1 );
        REQUIRE( vector[5] < 0.1 );
        REQUIRE( vector[5] > -0.1 );

        RMLVector<double> *rml_vector = new RMLVector<double> (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        setVector(rml_vector, vector);

        auto vector_result = Affine(rml_vector).vector_with_elbow(state.elbow_c[0], vector);

        REQUIRE( vector[0] == Approx(vector_result[0]) );
        REQUIRE( vector[1] == Approx(vector_result[1]) );
        REQUIRE( vector[2] == Approx(vector_result[2]) );
        REQUIRE( vector[3] == Approx(vector_result[3]) );
        REQUIRE( vector[4] == Approx(vector_result[4]) );
        REQUIRE( vector[5] == Approx(vector_result[5]) );
        REQUIRE( vector[6] == Approx(vector_result[6]) );

        auto pose_result = CartesianPose(rml_vector);

        for (int i = 0; i < 16; i++) {
            REQUIRE( pose.O_T_EE[i] == Approx(pose_result.O_T_EE[i]) );
        }

        REQUIRE( pose.elbow[0] == Approx(pose_result.elbow[0]) );
    }
}