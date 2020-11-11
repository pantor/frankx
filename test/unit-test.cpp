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
        auto affine_vector = affine.vector();

        REQUIRE( affine_vector[0] == Approx(0.48) );
        REQUIRE( affine_vector[1] == Approx(-0.204) );
        REQUIRE( affine_vector[2] == Approx(0.287) );
        REQUIRE( affine_vector[3] == Approx(1.2) );
        REQUIRE( affine_vector[4] == Approx(-0.25) );
        REQUIRE( affine_vector[5] == Approx(-2.06) );

        auto affine_copy = Affine(affine.array());
        auto affine_copy_vectory = affine_copy.vector();

        REQUIRE( affine_vector[0] == affine_copy_vectory[0] );
        REQUIRE( affine_vector[1] == affine_copy_vectory[1] );
        REQUIRE( affine_vector[2] == affine_copy_vectory[2] );
        REQUIRE( affine_vector[3] == affine_copy_vectory[3] );
        REQUIRE( affine_vector[4] == affine_copy_vectory[4] );
        REQUIRE( affine_vector[5] == affine_copy_vectory[5] );
    }
}