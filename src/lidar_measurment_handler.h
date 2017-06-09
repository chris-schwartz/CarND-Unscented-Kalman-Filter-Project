//
// Created by Chris Schwartz on 6/4/17.
//

#ifndef UNSCENTEDKF_LIDAR_MEASURMENT_HANDLER_H
#define UNSCENTEDKF_LIDAR_MEASURMENT_HANDLER_H

#include "measurement_handler.h"

class LidarMeasurementHandler : public MeasurementHandler {
 public:
    static LidarMeasurementHandler init(double noise_stdd_x_meters,
                                        double noise_stdd_y_meters);

    ~LidarMeasurementHandler();

    unique_ptr<VectorXd> CreateInitialStateVector(const MeasurementPackage &measurement_package) override;

    MatrixXd ComputeSigmaPointsInMeasurementSpace(const MatrixXd &Xsig_pred) override;

    VectorXd ExtractMeasurements(const VectorXd &Xsig_pred) override;

 private:

    LidarMeasurementHandler(const VectorXd &noise_std_deviation);
};

#endif //UNSCENTEDKF_LIDAR_MEASURMENT_HANDLER_H
