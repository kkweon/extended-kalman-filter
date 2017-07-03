//
// Created by kkweon on 6/14/17.
//
#include "catch.hpp"
#include "tools.h"

SCENARIO("RMSE can be calculated", "[Tools]") {
  GIVEN("two vectors with same length") {
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    VectorXd temp(4);
    temp << 1, 2, 3, 4;

    estimations.push_back(temp);
    ground_truth.push_back(temp);

    REQUIRE(estimations.size() == ground_truth.size());

    Tools tool;

    VectorXd expect(4);
    temp << 0, 0, 0, 0;

    WHEN("two vectors differ by {0, 0, 1, 1}") {
      estimations.clear();
      ground_truth.clear();

      temp << 0, 0, 0, 1;
      VectorXd temp2(4);
      temp2 << 0, 0, 1, 0;

      estimations.push_back(temp);
      ground_truth.push_back(temp2);

      THEN("it returns a VectorXd(4){0, 0, 1, 1}") {
        VectorXd expected(4);
        expected << 0, 0, 1, 1;

        REQUIRE(tool.CalculateRMSE(estimations, ground_truth) == expected);
      }
    }

    WHEN("two vectors differ by {4, 4, 4, 4}") {
      estimations.clear();
      ground_truth.clear();

      VectorXd single_estimate(4);
      single_estimate << 4, 4, 0, 0;
      estimations.push_back(single_estimate);

      VectorXd single_truth(4);
      single_truth << 0, 0, 4, 4;
      ground_truth.push_back(single_truth);

      THEN("it returns a VectorXd(4){4, 4, 4, 4}") {
        VectorXd expected(4);
        expected << 4, 4, 4, 4;

        REQUIRE(tool.CalculateRMSE(estimations, ground_truth) == expected);
      }

    }
  }
}

SCENARIO("jacobian matrix can be computed", "[Tools]") {
  GIVEN("a random vector") {
    Tools tools;
    VectorXd input(4);

    input << 1, 2, 0.2, 0.4;
    MatrixXd ret = tools.CalculateJacobian(input);

    THEN("a jacobian can be computed correctly") {
      MatrixXd expected(3, 4);
      expected << 0.4472136, 0.89442719, 0., 0.,
          -0.4, 0.2, 0., 0.,
          0., 0., 0.4472136, 0.89442719;

      expected -= ret;
      REQUIRE(expected.sum() < 1e-6);
    }
  }
}
