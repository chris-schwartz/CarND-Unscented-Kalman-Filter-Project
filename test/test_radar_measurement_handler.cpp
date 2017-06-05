//
// Created by Chris Schwartz on 6/3/17.
//

#include <radar_measurement_handler.h>
#include "test_radar_measurement_handler.h"
#include "eigen_assert.h"

TestRadarMeasurementHandler::TestRadarMeasurementHandler() : TestCase("TestRadarMeasurementHandler") {
    add_test("testCreatingInitialStateVector", &TestRadarMeasurementHandler::testCreatingInitialStateVector);
    add_test("testComputingNoiseCovarianceMatrix", &TestRadarMeasurementHandler::testComputingNoiseCovarianceMatrix);
    add_test("testPredictingMeasurements", &TestRadarMeasurementHandler::testPredictingMeasurements);
}

void TestRadarMeasurementHandler::testCreatingInitialStateVector() {

    RadarMeasurementHandler radar_measurement_handler = RadarMeasurementHandler::init(0.3, 0.0175, 0.1);

    MeasurementPackage measurement_package = GetMeasurementPackageForReadings(1.0, .55, 4.9);

    unique_ptr<VectorXd> result = radar_measurement_handler.CreateInitialStateVector(measurement_package);

    VectorXd expected(5);
    expected << 0.85252, 0.52269, 4.9, 0, 0;

    EigenAssert::assertMatricesEqual(*result, expected);
}

void TestRadarMeasurementHandler::testComputingNoiseCovarianceMatrix() {
    RadarMeasurementHandler radar_measurement_handler = RadarMeasurementHandler::init(2, 3, 4);

    MatrixXd result = radar_measurement_handler.ComputeNoiseCovarianceMatrix();

    MatrixXd expected(3, 3);
    expected << 4, 0, 0,
        0, 9, 0,
        0, 0, 16;

    EigenAssert::assertMatricesEqual(result, expected);
}

void TestRadarMeasurementHandler::testPredictingMeasurements() {
    RadarMeasurementHandler radar_measurement_handler = RadarMeasurementHandler::init(0.3, 0.0175, 0.1);

    //set vector for weights
    int n_aug = 7;
    double lambda = 3 - n_aug;
    VectorXd weights(15);
    double weight_0 = lambda / (lambda + n_aug);
    weights(0) = weight_0;
    for (int i = 1; i < 2 * n_aug + 1; i++) {
        double weight = 0.5 / (n_aug + lambda);
        weights(i) = weight;
    }

    MatrixXd Xsig_pred(5, 15);
    Xsig_pred
        << 5.9374, 6.0640, 5.925, 5.9436, 5.9266, 5.9374, 5.9389, 5.9374, 5.8106, 5.9457, 5.9310, 5.9465, 5.9374, 5.9359, 5.93744,
        1.48, 1.4436, 1.660, 1.4934, 1.5036, 1.48, 1.4868, 1.48, 1.5271, 1.3104, 1.4787, 1.4674, 1.48, 1.4851, 1.486,
        2.204, 2.2841, 2.2455, 2.2958, 2.204, 2.204, 2.2395, 2.204, 2.1256, 2.1642, 2.1139, 2.204, 2.204, 2.1702, 2.2049,
        0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337, 0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188, 0.5367, 0.535048,
        0.352, 0.29997, 0.46212, 0.37633, 0.4841, 0.41872, 0.352, 0.38744, 0.40562, 0.24347, 0.32926, 0.2214, 0.28687, 0.352, 0.318159;

    MeasurementPrediction result = radar_measurement_handler.PredictMeasurement(Xsig_pred, weights);

    VectorXd expected_z(3);
    MatrixXd expected_S(3, 3);

    expected_z << 6.12155, 0.245993, 2.10313;

    expected_S << 0.0946171, -0.000139448, 0.00407016,
        -0.000139448, 0.000617548, -0.000770652,
        0.00407016, -0.000770652, 0.0180917;

    EigenAssert::assertMatricesEqual(result.z_, expected_z);
    EigenAssert::assertMatricesEqual(result.S_, expected_S);
}

MeasurementPackage TestRadarMeasurementHandler::GetMeasurementPackageForReadings(double rho,
                                                                                 double phi,
                                                                                 double rhodot) const {
    MeasurementPackage measurement_package;

    measurement_package.sensor_type_ = MeasurementPackage::RADAR;
    VectorXd radar_measurements(3);

    radar_measurements << rho, phi, rhodot;

    measurement_package.raw_measurements_ = radar_measurements;
    return measurement_package;
}


