//
//  sigma_points.h
//  UnscentedKF
//
//  Created by Chris Schwartz on 5/25/17.
//
//


#ifndef sigma_point_operations_h
#define sigma_point_operations_h

#include "Eigen/Dense"

using namespace Eigen;

class SigmaPointOperations {
    
public:
    
    SigmaPointOperations(int row_count);
    
    MatrixXd predict_sigma_points(Eigen::MatrixXd sigma_points, double delta_t);
    
    MatrixXd generate_sigma_points(VectorXd x, MatrixXd P, double nu_acceleration, double nu_yawdd);
    
    VectorXd predict_mean(const MatrixXd& sigma_points);
    
    MatrixXd predict_covariance(const MatrixXd& sigma_points, const VectorXd& x);
    
private:
    
    VectorXd calculate_weights(int count);
    
    const int row_count_;
    const int augmented_row_count_;
    const int lambda_;
    
};

#endif /* sigma_points_h */
