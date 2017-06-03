
#include "test_sigma_point_operations.h"
#include "test_ukf.h"

int main(int argc, char* argv[]) {
    
    TestSigmaPointOperations test_sigma_point_operations;
    TestUKF test_ukf;
    
    test_sigma_point_operations.run_tests();
    test_ukf.run_tests();
    
    return 0;
    
}
