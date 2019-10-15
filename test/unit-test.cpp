#define CATCH_CONFIG_MAIN
#include <random>

#include <catch2/catch.hpp>

#include <frankx/affine.hpp>
#include <frankx/motion_waypoint.hpp>
#include <frankx/robot.hpp>
#include <frankx/utils.hpp>


using namespace frankx;


inline Affine getRelativeBase(double x = 0.0, double y = 0.0, double z = 0.0, double a = 0.0, double b = 0.0, double c = 0.0) {
  return Affine(0.48 + x, -0.204 + y, 0.267 + z, a, b, c);
}


TEST_CASE("Geometry") {
    SECTION("Basic transformations") {
        auto affine = getRelativeBase(0.0, 0.0, 0.02, 1.2, -0.25, -2.06);
        auto vector_result = affine.vector();

        REQUIRE( vector_result[0] == Approx(0.48) );
        REQUIRE( vector_result[1] == Approx(-0.204) );
        REQUIRE( vector_result[2] == Approx(0.287) );
        REQUIRE( vector_result[3] == Approx(1.2) );
        REQUIRE( vector_result[4] == Approx(-0.25) );
        REQUIRE( vector_result[5] == Approx(-2.06) );
    }

    SECTION("Robot state: RML Vector") {
        Robot robot("172.16.0.2");
        auto state = robot.readOnce();
        franka::CartesianPose pose = franka::CartesianPose(state.O_T_EE_c, state.elbow_c);

        auto vector = Affine(pose).vector(pose.elbow[0]);
        REQUIRE( vector[3] < 0.1 );
        REQUIRE( vector[3] > -0.1 );
        REQUIRE( vector[4] < 0.1 );
        REQUIRE( vector[4] > -0.1 );
        REQUIRE( vector[5] < 0.1 );
        REQUIRE( vector[5] > -0.1 );

        RMLVector<double> *rml_vector = new RMLVector<double> (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        setVector(rml_vector, vector);

        auto vector_result = Affine(rml_vector).vector(state.elbow_c[0], vector);

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

    SECTION("Random RML Vector") {
        std::default_random_engine generator;
        std::uniform_real_distribution<double> distribution(-3.14, 3.14);

        for (int i = 0; i < 500; i++) {
            double x = distribution(generator);
            double y = distribution(generator);
            double z = distribution(generator);
            double a = distribution(generator);
            double b = distribution(generator);
            double c = distribution(generator);
            double e = distribution(generator);

            RMLVector<double> *rml_vector = new RMLVector<double> (x, y, z, a, b, c, e);
            auto old_vector = Vector(rml_vector);

            auto vector_result = Affine(rml_vector).vector(rml_vector->VecData[6], old_vector);
            REQUIRE( rml_vector->VecData[0] == Approx(vector_result[0]) );
            REQUIRE( rml_vector->VecData[1] == Approx(vector_result[1]) );
            REQUIRE( rml_vector->VecData[2] == Approx(vector_result[2]) );
            REQUIRE( rml_vector->VecData[3] == Approx(vector_result[3]) );
            REQUIRE( rml_vector->VecData[4] == Approx(vector_result[4]) );
            REQUIRE( rml_vector->VecData[5] == Approx(vector_result[5]) );
            REQUIRE( rml_vector->VecData[6] == Approx(vector_result[6]) );
        }
    }
}