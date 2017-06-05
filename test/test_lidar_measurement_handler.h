//
// Created by Chris Schwartz on 6/4/17.
//

#ifndef UNSCENTEDKF_TEST_LIDAR_MEASUREMENT_HANDLER_H
#define UNSCENTEDKF_TEST_LIDAR_MEASUREMENT_HANDLER_H

#include <Eigen/Dense>
#include "test_case.h"

using Eigen::VectorXd;

class TestLidarMeasurementHandler : public TestCase<TestLidarMeasurementHandler> {

 public:
    TestLidarMeasurementHandler();

    void testCreatingInitialStateVector();

    void testComputingNoiseCovarianceMatrix();

    void testPredictingMeasurements();

    VectorXd GetWeights() const;
};

#endif //UNSCENTEDKF_TEST_LIDAR_MEASUREMENT_HANDLER_H
