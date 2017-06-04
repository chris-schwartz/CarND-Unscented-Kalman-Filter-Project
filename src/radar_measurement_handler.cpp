//
// Created by Chris Schwartz on 6/3/17.
//

#include "radar_measurement_handler.h"

RadarMeasurementHandler::RadarMeasurementHandler() : MeasurementHandler(3) {

}

RadarMeasurementHandler::~RadarMeasurementHandler() {

}

unique_ptr<VectorXd> RadarMeasurementHandler::CreateInitialStateVector(const MeasurementPackage &measurement_package) {
    VectorXd measurements = measurement_package.raw_measurements_;

    double rho = measurements(0);
    double phi = measurements(1);
    double rhodot = measurements(2);

    double px = rho * cos(phi);
    double py = rho * sin(phi);
    double vx = rhodot * cos(phi);
    double vy = rhodot * sin(phi);

    unique_ptr<VectorXd> x_ = unique_ptr<VectorXd>(new VectorXd(5));
    (*x_.get()) << px, py, sqrt(vx * vx + vy * vy), 0, 0;

    return x_;
}

MeasurementPrediction RadarMeasurementHandler::PredictMeasurement(const MatrixXd &Xsig_pred, const VectorXd &weights) {
    MatrixXd Zsig = BuildMeasurementSpaceMatrix(Xsig_pred.cols());

    long sigma_point_count = Zsig.cols();

    //transform sigma points into measurement space
    for (int i = 0; i < sigma_point_count; i++) {

        // extract values for better readability
        double p_x = Xsig_pred(0, i);
        double p_y = Xsig_pred(1, i);
        double v = Xsig_pred(2, i);
        double yaw = Xsig_pred(3, i);

        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;

        // measurement model
        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);  //r
        Zsig(1, i) = atan2(p_y, p_x);              //phi
        Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);   //r_dot
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
