//
//  measurement_handler.h
//  UnscentedKF
//
//  Created by Chris Schwartz on 5/31/17.
//
//



#ifndef measurement_handler_h
#define measurement_handler_h

#include <memory>

#include "measurement_package.h"

using namespace Eigen;
using namespace std;

struct MeasurementPrediction {
    MeasurementPrediction(VectorXd z, MatrixXd S) : z_(z), S_(S) {
    }

    VectorXd z_;
    MatrixXd S_;
    MatrixXd Tc_;
};

class MeasurementHandler {
    
public:

    MeasurementHandler(int measurement_dimension_count);

    virtual ~MeasurementHandler();

    virtual unique_ptr<VectorXd> CreateInitialStateVector(const MeasurementPackage &measurement_package) = 0;

    virtual MeasurementPrediction PredictMeasurement(const MatrixXd &Xsig_pred, const VectorXd &weights) = 0;

    void SetNoiseStandardDeviation(const VectorXd &noise_std_deviation);

    MatrixXd ComputeNoiseCovarianceMatrix();

 protected:

    MatrixXd BuildMeasurementSpaceMatrix(long column_count);

    long GetDimensionCount();

 private:

    long measurement_dimension_count_;

    VectorXd noise_std_deviation_;
};

#endif /* measurement_handler_h */
