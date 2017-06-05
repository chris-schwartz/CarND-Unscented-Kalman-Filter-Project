//
//  unit_test.h
//  UnscentedKF
//
//  Created by Chris Schwartz on 6/2/17.
//
//

#pragma once

#include <iostream>
#include <map>
#include <exception>
#include <ctime>
#include <chrono>

#include <unistd.h>

#include "runnable.h"
#include "test_assertions.h"

using namespace std;

template<typename T>  // T should always be a type that extends TestCase
class TestCase : public Runnable {

 public:

    typedef void(T::*TestFunctionPointer)();
    typedef std::map<std::string, TestFunctionPointer> TestMap;

    TestCase(std::string test_case_name) {
        test_case_name_ = test_case_name;
        before_all();
    }

    ~TestCase() {
        after_all();
    }

    virtual void before_all() {
        // do nothing by default
    }

    virtual void after_all() {
        // do nothing by default
    }

    virtual void before_each() {
        // do nothing by default
    }

    virtual void after_each() {
        // do nothing by default
    }

    void run() {
        cout << endl << "Running " << test_case_name_ << "..." << endl << endl;

        if (test_map_.size() < 1) {

            std::cout << endl << "ERROR: " << "No tests specified for test case: " << test_case_name_ << endl << endl;
            throw new FailedTestException("No tests specified for test case");
        }

        for (typename TestMap::iterator it = test_map_.begin(); it != test_map_.end(); ++it) {
            cout << "    Running " << it->first << "..." << endl;

            const long long start_time = current_time_milliseconds();

            std::string result = run_test(it->second);

            const double seconds_elapsed = (current_time_milliseconds() - start_time) / 1000.0;

            std::cout << "    " << it->first << " -> " << result << " (" << seconds_elapsed << " s)" << endl << endl;
        }
    }

 protected:

    void add_test(const std::string &test_name, void(T::*test_ptr)()) {
        test_map_[test_name] = test_ptr;
    }

 private:

    std::string run_test(TestFunctionPointer ptr) {
        std::string result;
        try {
            (*dynamic_cast<T *>(this).*ptr)();
            result = "PASSED";
            passed_tests_++;
        }
        catch (exception &e) {
            result = "FAILED";
            std::cout << endl << "  TEST FAILURE: " << e.what() << endl << endl;
        }

        return result;
    }

    long long current_time_milliseconds() {
        return chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();
    }

    TestMap test_map_;
    int passed_tests_;

    std::string test_case_name_;

};

