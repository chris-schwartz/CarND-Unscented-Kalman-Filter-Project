//
// Created by Chris Schwartz on 6/3/17.
//

#include <iostream>
#include "measurement_handler.h"

MeasurementHandler::MeasurementHandler(int measurement_dimension_count, const VectorXd &noise_stdd) :
    measurement_dimension_count_(measurement_dimension_count), noise_std_deviation_(noise_stdd) {

}

MeasurementHandler::~MeasurementHandler() {

}

MatrixXd MeasurementHandler::BuildMeasurementSpaceMatrix(long column_count) {
    MatrixXd measurement_space_matrix(measurement_dimension_count_, column_count);
    measurement_space_matrix.setZero();
    return measurement_space_matrix;
}

long MeasurementHandler::GetDimensionCount() {
    return measurement_dimension_count_;
}

MatrixXd MeasurementHandler::ComputeNoiseCovarianceMatrix() {
    MatrixXd R = MatrixXd::Zero(measurement_dimension_count_, measurement_dimension_count_);

    long current_column = 0;
    for (long row = 0; row < measurement_dimension_count_; row++) {
        R(row, current_column) = noise_std_deviation_(row) * noise_std_deviation_(row);
        current_column++;
    }
    return R;
}

MeasurementPrediction MeasurementHandler::PredictMeasurement(const MatrixXd &Xsig_pred,
                                                             const VectorXd &weights) {
    MatrixXd Zsig = ComputeSigmaPointsInMeasurementSpace(Xsig_pred);
    long sigma_point_count = Zsig.cols();

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

void MeasurementHandler::Update(const MeasurementPrediction &measurement_prediction,
                                const VectorXd &raw_easurements,
                                const MatrixXd &Xsig_pred,
                                const VectorXd &weights,
                                VectorXd &x_out,
                                MatrixXd &P_out) {

    auto n_z = GetDimensionCount();

    auto z_pred = measurement_prediction.z_;
    auto S = measurement_prediction.S_;

    auto Zsig = ComputeSigmaPointsInMeasurementSpace(Xsig_pred);
    auto sigma_point_count = Zsig.cols();

    auto z = ExtractMeasurements(raw_easurements);

    MatrixXd Tc(5, n_z);
    Tc.fill(0.0);

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < sigma_point_count; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x_out;
        //angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        Tc = Tc + weights(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = z - z_pred;

    //angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    //update state mean and covariance matrix
    x_out = x_out + K * z_diff;
    P_out = P_out - K * S * K.transpose();
}
