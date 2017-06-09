//
// Created by Chris Schwartz on 6/3/17.
//

#include "radar_measurement_handler.h"

RadarMeasurementHandler::RadarMeasurementHandler(const VectorXd &noise_stdd) : MeasurementHandler(3, noise_stdd) {

}

RadarMeasurementHandler::~RadarMeasurementHandler() {

}

RadarMeasurementHandler RadarMeasurementHandler::init(double noise_stdd_radius_meters,
                                                      double noise_stdd_angle_radians,
                                                      double noise_stdd_radius_meters_per_sec) {
    VectorXd noise_stdd(3);
    noise_stdd << noise_stdd_radius_meters,
        noise_stdd_angle_radians,
        noise_stdd_radius_meters_per_sec;

    return RadarMeasurementHandler(noise_stdd);
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

MatrixXd RadarMeasurementHandler::ComputeSigmaPointsInMeasurementSpace(const MatrixXd &Xsig_pred) {
    long sigma_point_count = Xsig_pred.cols();
    MatrixXd Zsig = BuildMeasurementSpaceMatrix(sigma_point_count);

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

    return Zsig;
}

VectorXd RadarMeasurementHandler::ExtractMeasurements(const VectorXd &raw_measurements) {
    VectorXd z(3);
    z << raw_measurements(0), raw_measurements(1), raw_measurements(2);
    return z;
}
