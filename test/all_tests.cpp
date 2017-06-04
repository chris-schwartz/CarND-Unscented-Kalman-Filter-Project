
#include <memory>

#include "test_sigma_point_operations.h"
#include "test_ukf.h"
#include "test_radar_measurement_handler.h"

void add_test_case(Runnable *test, vector<unique_ptr<Runnable>> &test_cases_out) {
    test_cases_out.push_back(unique_ptr<Runnable>(test));
}

int main(int argc, char* argv[]) {
    vector<unique_ptr<Runnable>> test_cases;

    add_test_case(new TestSigmaPointOperations(), test_cases);
    add_test_case(new TestUKF(), test_cases);
    add_test_case(new TestRadarMeasurementHandler(), test_cases);

    for (auto &test_case : test_cases) {
        test_case->run();
    }

    return 0;

}
