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

MeasurementPrediction LidarMeasurementHandler::PredictMeasurement(const MatrixXd &Xsig_pred, const VectorXd &weights) {
    MatrixXd Zsig = BuildMeasurementSpaceMatrix(Xsig_pred.cols());

    long sigma_point_count = Zsig.cols();

    // TODO, should be able to pull most of the method up into superclass, and write specific implementaion for
    // TODO transofrming simga points into measurement space for each handler.

    //transform sigma points into measurement space
    for (int i = 0; i < sigma_point_count; i++) {
        // measurement model
        Zsig(0, i) = Xsig_pred(0, i);  // px
        Zsig(1, i) = Xsig_pred(1, i);  // py
    }

    long n_z = GetDimensionCount();

    //mean predicted measurement
    VectorXd z_pred(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < sigma_point_count; i++) {
        z_pred = z_pred + weights(i) * Zsig.col(i);
    }

    //measurement covariance matrix S
    MatrixXd S(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < sigma_point_count; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        S = S + weights(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    S = S + ComputeNoiseCovarianceMatrix();

    MeasurementPrediction prediction(z_pred, S);
    return prediction;
}
LidarMeasurementHandler LidarMeasurementHandler::init(double noise_stdd_x_meters,
                                                      double noise_stdd_y_meters) {
    VectorXd noise_stdd(2);
    noise_stdd << noise_stdd_x_meters,
        noise_stdd_y_meters;

    return LidarMeasurementHandler(noise_stdd);
}
