//
// Created by Chris Schwartz on 6/3/17.
//

#include "measurement_handler.h"

MeasurementHandler::MeasurementHandler(int measurement_dimension_count) : measurement_dimension_count_(
    measurement_dimension_count), noise_std_deviation_(VectorXd::Zero(measurement_dimension_count)) {

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

void MeasurementHandler::SetNoiseStandardDeviation(const VectorXd &noise_std_deviation) {
    noise_std_deviation_ = noise_std_deviation;
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
