//
// Created by Chris Schwartz on 6/4/17.
//

#include "lidar_measurment_handler.h"

LidarMeasurementHandler::LidarMeasurementHandler(const VectorXd &noise_std_deviation) : MeasurementHandler(2,
                                                                                                           noise_std_deviation) {

}

LidarMeasurementHandler::~LidarMeasurementHandler() {

}

unique_ptr<VectorXd> LidarMeasurementHandler::CreateInitialStateVector(const MeasurementPackage &measurement_package) {
    auto x_ptr = unique_ptr<VectorXd>(new VectorXd(5));
    *x_ptr.get() << measurement_package.raw_measurements_(0),
        measurement_package.raw_measurements_(1),
        0,
        0,
        0;

    return x_ptr;
}

LidarMeasurementHandler LidarMeasurementHandler::init(double noise_stdd_x_meters,
                                                      double noise_stdd_y_meters) {
    VectorXd noise_stdd(2);
    noise_stdd << noise_stdd_x_meters,
        noise_stdd_y_meters;

    return LidarMeasurementHandler(noise_stdd);
}

MatrixXd LidarMeasurementHandler::ComputeSigmaPointsInMeasurementSpace(const MatrixXd &Xsig_pred) {
    MatrixXd Zsig = BuildMeasurementSpaceMatrix(Xsig_pred.cols());

    //transform sigma points into measurement space
    for (int i = 0; i < Zsig.cols(); i++) {
        // measurement model
        Zsig(0, i) = Xsig_pred(0, i);  // px
        Zsig(1, i) = Xsig_pred(1, i);  // py
    }

    return Zsig;
}

VectorXd LidarMeasurementHandler::ExtractMeasurements(const VectorXd &raw_measurements) {
    VectorXd z(2);
    z << raw_measurements(0), raw_measurements(1);
    return z;
}
