
#include <memory>

#include "test_sigma_point_operations.h"
#include "test_ukf.h"
#include "test_radar_measurement_handler.h"
#include "test_lidar_measurement_handler.h"
#include "test_measurement_handler.h"

void add_test_case(Runnable *test, vector<unique_ptr<Runnable>> &test_cases_out) {
    test_cases_out.push_back(unique_ptr<Runnable>(test));
}

long long current_time_milliseconds() {
    return chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();
}

int main(int argc, char* argv[]) {
    vector<unique_ptr<Runnable>> test_cases;

    add_test_case(new TestSigmaPointOperations(), test_cases);
    add_test_case(new TestUKF(), test_cases);

    add_test_case(new TestRadarMeasurementHandler(), test_cases);
    add_test_case(new TestLidarMeasurementHandler(), test_cases);

    add_test_case(new TestMeasurementHandler(), test_cases);

    auto start_time = current_time_milliseconds();

    for (auto &test_case : test_cases) {
        test_case->run();
    }

    auto seconds_elapsed = (current_time_milliseconds() - start_time) / 1000.0;

    std::cout << endl << "Finished running all tests in " << seconds_elapsed << " s";

    return 0;

}
