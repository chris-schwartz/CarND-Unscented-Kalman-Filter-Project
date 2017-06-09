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

class MockMeasurementHandler : public MeasurementHandler {

public:
    bool create_initial_state_vector_called_ = false;
    bool computes_sigma_points_ = false;

    MockMeasurementHandler() : MeasurementHandler(3, VectorXd::Zero(3)) {

    }

    unique_ptr<VectorXd> CreateInitialStateVector(const MeasurementPackage &measurement_package) {
        create_initial_state_vector_called_ = true;
        return unique_ptr<VectorXd>(new VectorXd(5));
    }

    MatrixXd ComputeSigmaPointsInMeasurementSpace(const MatrixXd &Xsig_pred) {
        computes_sigma_points_ = true;
        auto Zsig = BuildMeasurementSpaceMatrix(Xsig_pred.cols());
        return MatrixXd::Zero(Zsig.rows(), Zsig.cols());
    }

    VectorXd ExtractMeasurements(const VectorXd &raw_measurements) {
        return VectorXd::Zero(3);
    }

};

class TestUKF : public TestCase<TestUKF> {
public:

    TestUKF();

    void testRegisteringMeasurementHandlers();

};
#endif /* test_ukf_h */
