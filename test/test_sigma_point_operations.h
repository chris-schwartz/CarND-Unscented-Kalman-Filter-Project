//
//  test_sigma_point_operations.h
//  UnscentedKF
//
//  Created by Chris Schwartz on 6/2/17.
//
//


#ifndef test_sigma_point_operations_h
#define test_sigma_point_operations_h

#include "test_case.h"
#include "sigma_point_operations.h"
#include "eigen_asserts.h"

class TestSigmaPointOperations : public TestCase<TestSigmaPointOperations> {
    
public:
    
    TestSigmaPointOperations() {
        
        add_test("testGeneratingSigmaPoints", &TestSigmaPointOperations::testGeneratingSigmaPoints);
        add_test("testPredictingSigmaPoints", &TestSigmaPointOperations::testPredictingSigmaPoints);
        add_test("testPredictingMean", &TestSigmaPointOperations::testPredictingMean);
        add_test("testPredictingCovariance", &TestSigmaPointOperations::testPredictingCovariance);

    }
    
    ~TestSigmaPointOperations() {
        
    }
    
    void testGeneratingSigmaPoints() {
        VectorXd x = VectorXd(5);
        x <<   5.7441,
        1.3800,
        2.2049,
        0.5015,
        0.3528;
        
        //create example covariance matrix
        MatrixXd P = MatrixXd(5, 5);
        P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
        -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
        0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
        -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
        -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
        
        
        SigmaPointOperations sigma_pt_ops(5);
        auto result_points = sigma_pt_ops.generate_sigma_points(x, P, 0.2, 0.2);
        
        MatrixXd expected_points(7, 15);
        expected_points <<
        5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,    5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441, 5.7441, 5.7441,
        1.38,    1.34566,   1.52806,    1.38,     1.38,     1.38,    1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38,    1.38,     1.38,
        2.2049,  2.28414,  2.24557,  2.29582,   2.2049,    2.2049,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
        0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
        0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
        0,        0,        0,        0,       0,        0,  0.34641,        0,        0,        0,        0,        0,        0, -0.34641,        0,
        0,        0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0,        0, -0.34641;
        
        EigenAssert::assertMatricesEqual(result_points, expected_points);
    }
    
    
    void testPredictingSigmaPoints() {
        
        MatrixXd sigma_points(7, 15);
        sigma_points <<
        5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,    5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441, 5.7441, 5.7441,
        1.38,    1.34566,   1.52806,    1.38,     1.38,     1.38,    1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38,    1.38,     1.38,
        2.2049,  2.28414,  2.24557,  2.29582,   2.2049,    2.2049,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
        0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
        0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
        0,        0,        0,        0,       0,        0,  0.34641,        0,        0,        0,        0,        0,        0, -0.34641,        0,
        0,        0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0,        0, -0.34641;
        
        SigmaPointOperations sigma_point_ops(5);
        MatrixXd predicted_pts = sigma_point_ops.predict_sigma_points(sigma_points, 0.1);
        
        MatrixXd expected_predicted_pts = MatrixXd(5, 15);
        expected_predicted_pts <<
        5.93553, 6.06251, 5.92217, 5.9415, 5.92361, 5.93516, 5.93705, 5.93553, 5.80832, 5.94481, 5.92935, 5.94553, 5.93589, 5.93401, 5.93553,
        1.48939, 1.44673, 1.66484, 1.49719, 1.508, 1.49001, 1.49022, 1.48939, 1.5308, 1.31287, 1.48182, 1.46967, 1.48876, 1.48855, 1.48939,
        2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.23954, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.17026, 2.2049,
        0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
        0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.387441, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.318159;
        
        EigenAssert::assertMatricesEqual(predicted_pts, expected_predicted_pts);
    }
    
    
    void testPredictingMean() {
        MatrixXd sigma_points = MatrixXd(5,15);
        sigma_points <<
        5.93553, 6.06251, 5.92217, 5.9415, 5.92361, 5.93516, 5.93705, 5.93553, 5.80832, 5.94481, 5.92935, 5.94553, 5.93589, 5.93401, 5.93553,
        1.48939, 1.44673, 1.66484, 1.49719, 1.508, 1.49001, 1.49022, 1.48939, 1.5308, 1.31287, 1.48182, 1.46967, 1.48876, 1.48855, 1.48939,
        2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.23954, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.17026, 2.2049,
        0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
        0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.387441, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.318159;
        
        SigmaPointOperations sigma_pt_ops(5);
        VectorXd predicted_mean = sigma_pt_ops.predict_mean(sigma_points);
        
        VectorXd expected_mean = VectorXd(5);
        expected_mean <<
        5.93637,
        1.49035,
        2.20528,
        0.536853,
        0.353577;
        
        EigenAssert::assertMatricesEqual(predicted_mean, expected_mean);
    }
    
    
    void testPredictingCovariance() {
        
        MatrixXd sigma_points = MatrixXd(5,15);
        sigma_points <<
        5.93553, 6.06251, 5.92217, 5.9415, 5.92361, 5.93516, 5.93705, 5.93553, 5.80832, 5.94481, 5.92935, 5.94553, 5.93589, 5.93401, 5.93553,
        1.48939, 1.44673, 1.66484, 1.49719, 1.508, 1.49001, 1.49022, 1.48939, 1.5308, 1.31287, 1.48182, 1.46967, 1.48876, 1.48855, 1.48939,
        2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.23954, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.17026, 2.2049,
        0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
        0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.387441, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.318159;
        
        VectorXd mean = VectorXd(5);
        mean <<
        5.93637,
        1.49035,
        2.20528,
        0.536853,
        0.353577;
        
        SigmaPointOperations sigma_pt_ops = SigmaPointOperations(5);
        MatrixXd predicted_covariance = sigma_pt_ops.predict_covariance(sigma_points, mean);
        
        MatrixXd expected_covariance = MatrixXd(5,5);
        expected_covariance <<
        0.00543425, -0.0024053, 0.00341576, -0.00348196, -0.00299378,
        -0.0024053, 0.010845, 0.0014923, 0.00980182, 0.00791091,
        0.00341576, 0.0014923, 0.00580129, 0.000778632, 0.000792973,
        -0.00348196, 0.00980182, 0.000778632, 0.0119238, 0.0112491,
        -0.00299378, 0.00791091, 0.000792973, 0.0112491, 0.0126972;
        
        EigenAssert::assertMatricesEqual(predicted_covariance, expected_covariance);
    }

    
};

#endif /* test_sigma_point_operations_h */
