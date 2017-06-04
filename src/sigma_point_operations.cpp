//
//  sigma_point_operations.cpp
//  UnscentedKF
//
//  Created by Chris Schwartz on 5/26/17.
//
//
#include "sigma_point_operations.h"

using namespace Eigen;


SigmaPointOperations::SigmaPointOperations(int row_count) : row_count_(row_count), augmented_row_count_(row_count_ + 2), lambda_(3 - augmented_row_count_) {
    
}

MatrixXd SigmaPointOperations::predict_sigma_points(Eigen::MatrixXd sigma_points, double delta_t) {
    int sigma_point_count = 2 * augmented_row_count_ + 1;
    
    //create matrix with predicted sigma points as columns
    MatrixXd predicted_points = MatrixXd(row_count_, sigma_point_count);
    
    //predict sigma points
    for (int i = 0; i< sigma_point_count; i++)
    {
        int row_idx = 0;
        //extract values for better readability
        double p_x = sigma_points(row_idx++,i);
        double p_y = sigma_points(row_idx++,i);
        double v = sigma_points(row_idx++,i);
        double yaw = sigma_points(row_idx++,i);
        double yawd = sigma_points(row_idx++,i);
        double nu_a = sigma_points(row_idx++,i);
        double nu_yawdd = sigma_points(row_idx++,i);
        
        //predicted state values
        double px_p, py_p;
        
        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;
        
        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;
        
        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;
        
        //write predicted sigma point into right column
        row_idx = 0;
        predicted_points(row_idx++,i) = px_p;
        predicted_points(row_idx++,i) = py_p;
        predicted_points(row_idx++,i) = v_p;
        predicted_points(row_idx++,i) = yaw_p;
        predicted_points(row_idx++,i) = yawd_p;
    }
    
    return predicted_points;
}

MatrixXd SigmaPointOperations::generate_sigma_points(VectorXd x, MatrixXd P, double nu_acceleration, double nu_yawdd) {
    
    int n = row_count_;
    int n_aug = augmented_row_count_;
    
    auto x_aug = VectorXd(augmented_row_count_);
    x_aug.setZero();
    x_aug.head(n) = x;
    x_aug[n] = 0;
    x_aug[n+1] = 0;
    
    auto P_aug = MatrixXd(n+2, n+2);
    P_aug.setZero();
    P_aug.topLeftCorner(n, n) = P;
    P_aug(n,n) = nu_acceleration * nu_acceleration;
    P_aug(n+1, n+1) = nu_yawdd * nu_yawdd;
    
    //calculate sigma points
    auto sigma_points = MatrixXd(n_aug, 2*n_aug+1);
    MatrixXd L = P_aug.llt().matrixL(); // sqrt matrix
    
    sigma_points.col(0)  = x_aug;
    for (int i = 0; i< n_aug; i++)
    {
        sigma_points.col(i+1)       = x_aug + sqrt(lambda_+n_aug) * L.col(i);
        sigma_points.col(i+1+n_aug) = x_aug - sqrt(lambda_+n_aug) * L.col(i);
    }
    
    return sigma_points;
}

VectorXd SigmaPointOperations::predict_mean(const MatrixXd& sigma_points) {
    int sigma_point_count = sigma_points.cols();

    VectorXd x(row_count_);
    x.fill(0.0);
    
    VectorXd weights = calculate_weights(sigma_point_count);
    
    //predicted state mean
    for (int i = 0; i < sigma_point_count; i++) {
        x = x + weights(i) * sigma_points.col(i);
    }
    return x;
}

MatrixXd SigmaPointOperations::predict_covariance(const MatrixXd& sigma_points, const VectorXd& x) {
    int sigma_point_count = sigma_points.cols();
    VectorXd weights = calculate_weights(sigma_point_count);
    MatrixXd P = MatrixXd(row_count_, row_count_);
    
    P.fill(0.0);
    for (int i = 0; i < sigma_point_count; i++) {  //iterate over sigma points
        
        // state difference
        VectorXd x_diff = sigma_points.col(i) - x;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
        
        P = P + weights(i) * x_diff * x_diff.transpose() ;
    }
    return P;
    
}


VectorXd SigmaPointOperations::calculate_weights(int sigma_point_count) {

    VectorXd weights(sigma_point_count);

    weights(0) = (lambda_ * 1.0) / (lambda_ + augmented_row_count_);
    for (int i=1; i<sigma_point_count; i++) {
        weights(i) = 0.5/(augmented_row_count_+lambda_);
    }
    
    return weights;
}
