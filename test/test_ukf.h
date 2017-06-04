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

    TestMeasurementHandler() : MeasurementHandler(1) {

    }

    unique_ptr<VectorXd> CreateInitialStateVector(const MeasurementPackage &measurement_package) {
        create_initial_state_vector_called_ = true;
        return unique_ptr<VectorXd>(new VectorXd(5));
    }

    MeasurementPrediction PredictMeasurement(const MatrixXd &Xsig_pred, const VectorXd &weights) {
        process_measurement_called_ = true;
        return MeasurementPrediction(VectorXd(1), MatrixXd(1, 1));
    }
    
};

class TestUKF : public TestCase<TestUKF> {
public:

    TestUKF();

    void testRegisteringMeasurementHandlers();

};
#endif /* test_ukf_h */
