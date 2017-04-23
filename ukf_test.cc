#include "libs/catch.hpp"
#include "ukf.h"

TEST_CASE("UFK TESTS", "[sigmapoints]") {
  Ukf ufk;

  SECTION("GenerateSigmaPoints() Test") {
    REQUIRE(false);
  }

  SECTION("PredictSigmaPoints() Test") {
    REQUIRE(false);
  }

  SECTION("PredictMeanAndCovariance() Test") {
    REQUIRE(false);
  }

  SECTION("PredictMeasurement() Test") {
    REQUIRE(false);
  }

  SECTION("Update() Test") {
    REQUIRE(false);
  }
}

SCENARIO("A sensor reading is sent to ProcessMeasurement()", "[use_radar_][use_laser_]") {
  GIVEN( "A LIDAR measurement" ) {
    WHEN( "the use_laser_ is set to false" ) {
      THEN( "the state should not be updated." ) {
        REQUIRE(false);
      }
    }

    WHEN( "the use_laser_ is set to true" ) {
      THEN( "the state should be updated." ) {
        REQUIRE(false);
      }
    }
  }

  GIVEN( "A RADAR measurement" ) {
    WHEN( "the use_radar_ is set to false" ) {
      THEN( "the state should not be updated." ) {
        REQUIRE(false);
      }
    }

    WHEN( "the use_radar_ is set to true" ) {
      THEN( "the state should be updated." ) {
        REQUIRE(false);
      }
    }
  }
}
