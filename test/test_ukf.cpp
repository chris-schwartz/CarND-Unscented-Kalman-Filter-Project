//
// Created by Chris Schwartz on 6/3/17.
//
#include "test_ukf.h"

TestUKF::TestUKF() : TestCase("TestUKF") {
    add_test("testRegisteringMeasurementHandlers", &TestUKF::testRegisteringMeasurementHandlers);
}

void TestUKF::testRegisteringMeasurementHandlers() {
    UKF ukf;

    MockMeasurementHandler *testHandler = new MockMeasurementHandler();
    ukf.RegisterMeasurementHandler(MeasurementPackage::SensorType::RADAR, testHandler);

    MeasurementPackage laser_package;
    MeasurementPackage radar_package;

    laser_package.sensor_type_ = MeasurementPackage::SensorType::LASER;
    radar_package.sensor_type_ = MeasurementPackage::SensorType::RADAR;

    ukf.ProcessMeasurement(laser_package);
    TestAssertions::assertFalse(testHandler->computes_sigma_points_);
    TestAssertions::assertFalse(testHandler->create_initial_state_vector_called_);

    // Initializes creating initial state vector the first its called
    ukf.ProcessMeasurement(radar_package);
    TestAssertions::assertFalse(testHandler->computes_sigma_points_);
    TestAssertions::assertTrue(testHandler->create_initial_state_vector_called_);

    // Processes new measurement the second time
    ukf.ProcessMeasurement(radar_package);
    TestAssertions::assertTrue(testHandler->computes_sigma_points_);
}

