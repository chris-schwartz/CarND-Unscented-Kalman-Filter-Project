//
//  test_ukf.h
//  UnscentedKF
//
//  Created by Chris Schwartz on 6/3/17.
//
//

#ifndef test_ukf_h
#define test_ukf_h

#include <memory>

#include "ukf.h"
#include "test_case.h"


class TestMeasurementHandler : public MeasurementHandler {

public:
    bool create_initial_state_vector_called_ = false;
    bool process_measurement_called_ = false;
    
    unique_ptr<VectorXd> CreateInitialStateVector(MeasurementPackage measurement_package) {
        create_initial_state_vector_called_ = true;
        return unique_ptr<VectorXd>(new VectorXd(5));
    }
    
    void ProcessMeasurement(MeasurementPackage measurement_package) {
        process_measurement_called_ = true;
        return;
    }
    
};

class TestUKF : public TestCase<TestUKF> {
public:
    
    TestUKF() {
        add_test("testRegisteringMeasurementHandlers", &TestUKF::testRegisteringMeasurementHandlers);
    }
    
    void testRegisteringMeasurementHandlers() {
        UKF ukf;
        
        TestMeasurementHandler* testHandler = new TestMeasurementHandler();
        ukf.RegisterMeasurementHandler(MeasurementPackage::SensorType::RADAR, testHandler);
        
        MeasurementPackage laser_package;
        MeasurementPackage radar_package;
        
        laser_package.sensor_type_ = MeasurementPackage::SensorType::LASER;
        radar_package.sensor_type_ = MeasurementPackage::SensorType::RADAR;
        
        ukf.ProcessMeasurement(laser_package);
        TestAssertions::assertFalse(testHandler->process_measurement_called_);
        
        ukf.ProcessMeasurement(radar_package);
        TestAssertions::assertTrue(testHandler->process_measurement_called_);
    }
};
#endif /* test_ukf_h */
