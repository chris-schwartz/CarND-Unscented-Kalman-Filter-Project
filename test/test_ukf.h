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

    TestUKF();

    void testRegisteringMeasurementHandlers();

};
#endif /* test_ukf_h */
