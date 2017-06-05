//
// Created by Chris Schwartz on 6/4/17.
//

#include "test_lidar_measurement_handler.h"
#include "lidar_measurment_handler.h"
#include "eigen_assert.h"

using namespace std;

TestLidarMeasurementHandler::TestLidarMeasurementHandler() : TestCase("TestLidarMeasurementHandler") {
    add_test("testCreatingInitialStateVector", &TestLidarMeasurementHandler::testCreatingInitialStateVector);
    add_test("testComputingNoiseCovarianceMatrix", &TestLidarMeasurementHandler::testComputingNoiseCovarianceMatrix);
    add_test("testPredictingMeasurements", &TestLidarMeasurementHandler::testPredictingMeasurements);
}

void TestLidarMeasurementHandler::testCreatingInitialStateVector() {
    LidarMeasurementHandler handler = LidarMeasurementHandler::init(0, 0);

    MeasurementPackage lidar_measurement_package;
    lidar_measurement_package.sensor_type_ = MeasurementPackage::SensorType::LASER;

    VectorXd measurements(5);
    measurements << 0.1234, 5.6789, 0.001, 0.01, 0.1;

    lidar_measurement_package.raw_measurements_ = measurements;

    auto result = handler.CreateInitialStateVector(lidar_measurement_package);

    VectorXd expected(5);
    expected << 0.1234, 5.6789, 0.0, 0.0, 0.0;

    EigenAssert::assertMatricesEqual(*result.get(), expected);
}

void TestLidarMeasurementHandler::testComputingNoiseCovarianceMatrix() {
    LidarMeasurementHandler handler = LidarMeasurementHandler::init(2, 3);

    MatrixXd result = handler.ComputeNoiseCovarianceMatrix();

    MatrixXd expected_result(2, 2);
    expected_result << 4, 0,
        0, 9;

    EigenAssert::assertMatricesEqual(result, expected_result);
}

void TestLidarMeasurementHandler::testPredictingMeasurements() {
    LidarMeasurementHandler handler = LidarMeasurementHandler::init(0.15, 0.15);

    MatrixXd Xsig_pred(5, 15);
    Xsig_pred
        << 5.9374, 6.0640, 5.925, 5.9436, 5.9266, 5.9374, 5.9389, 5.9374, 5.8106, 5.9457, 5.9310, 5.9465, 5.9374, 5.9359, 5.93744,
        1.48, 1.4436, 1.660, 1.4934, 1.5036, 1.48, 1.4868, 1.48, 1.5271, 1.3104, 1.4787, 1.4674, 1.48, 1.4851, 1.486,
        2.204, 2.2841, 2.2455, 2.2958, 2.204, 2.204, 2.2395, 2.204, 2.1256, 2.1642, 2.1139, 2.204, 2.204, 2.1702, 2.2049,
        0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337, 0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188, 0.5367, 0.535048,
        0.352, 0.29997, 0.46212, 0.37633, 0.4841, 0.41872, 0.352, 0.38744, 0.40562, 0.24347, 0.32926, 0.2214, 0.28687, 0.352, 0.318159;

    VectorXd weights = GetWeights();

    MeasurementPrediction results = handler.PredictMeasurement(Xsig_pred, weights);

    VectorXd expected_z(2);
    expected_z << 5.93637,
        1.49035;

    EigenAssert::assertMatricesEqual(results.z_, expected_z);

    MatrixXd expected_S(2, 2);
    expected_S << 0.0279342, -0.0024053,
        -0.0024053, 0.033345;

    EigenAssert::assertMatricesEqual(results.S_, expected_S);

}

VectorXd TestLidarMeasurementHandler::GetWeights() const {
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
    return weights;
}
