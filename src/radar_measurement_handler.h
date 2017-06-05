//
// Created by Chris Schwartz on 6/3/17.
//

#pragma once

#include "Eigen/Dense"

#include "measurement_handler.h"
#include "measurement_package.h"

class RadarMeasurementHandler : public MeasurementHandler {

 public:
    ~RadarMeasurementHandler() override;

    static RadarMeasurementHandler init(double noise_stdd_radius_meters,
                                        double noise_stdd_angle_radians,
                                        double noise_stdd_radius_meters_per_sec);

    unique_ptr<VectorXd> CreateInitialStateVector(const MeasurementPackage &measurement_package) override;

    MeasurementPrediction PredictMeasurement(const MatrixXd &Xsig_pred,
                                             const VectorXd &weights) override;

 private:
    RadarMeasurementHandler(const VectorXd &noise_stdd);

};

