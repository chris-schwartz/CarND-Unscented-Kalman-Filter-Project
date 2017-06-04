//
//  eigen_assertions.h
//  UnscentedKF
//
//  Created by Chris Schwartz on 6/3/17.
//
//

#ifndef eigen_assertions_h
#define eigen_assertions_h
#include <exception>
#include <sstream>
#include "test_assertions.h"
#include "Eigen/Dense"

class EigenAssert {
    
public:
    
static void assertMatricesEqual(const MatrixXd& actual, const MatrixXd& expected, double tolerance = 0.0001) {
    for (long r = 0; r < expected.rows(); r++) {
        for (long c = 0; c < expected.cols(); c++) {
            auto diff = abs(expected(r, c) - actual(r, c));
            if(diff >= tolerance) {
                std::ostringstream os;
                os << "Difference of " << diff << " found at row: " << r << " column: " << c;
                throw FailedTestException(os.str());
            }
        }
    }
}
    
};

#endif /* eigen_assertions_h */
