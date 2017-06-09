//
// Created by Chris Schwartz on 6/3/17.
//

#ifndef UNSCENTEDKF_TEST_RADAR_MEASUREMENT_HANDLER_H
#define UNSCENTEDKF_TEST_RADAR_MEASUREMENT_HANDLER_H

#include "test_case.h"

class TestRadarMeasurementHandler : public TestCase<TestRadarMeasurementHandler> {

 public:

    TestRadarMeasurementHandler();

    void testCreatingInitialStateVector();

    void testComputingNoiseCovarianceMatrix();

    void testPredictingMeasurements();

    void testExtractingMeasurements();

 private:

    MeasurementPackage GetMeasurementPackageForReadings(double rho, double phi, double rhodot) const;
};

#endif //UNSCENTEDKF_TEST_RADAR_MEASUREMENT_HANDLER_H
