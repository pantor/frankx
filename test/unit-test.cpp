#define CATCH_CONFIG_MAIN
#include <random>

#include <catch2/catch.hpp>

#include <frankx/frankx.hpp>


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

            auto vector_result = Affine(rml_vector).vector_with_elbow(rml_vector->VecData[6], old_vector);
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