//
//  test_assertions.h
//  UnscentedKF
//
//  Created by Chris Schwartz on 6/3/17.
//
//

#ifndef test_assertions_h
#define test_assertions_h
#include <string>

struct FailedTestException : public std::exception {
    
    const std::string reason;
    
    FailedTestException(std::string reason) : reason(reason) {
    }
    
    ~FailedTestException() throw () {} // Updated
    const char* what() const throw() { return reason.c_str(); }
};

class TestAssertions {
public:
    
    static void assertFalse(bool value, std::string reason = "") {
        if(value != false) {
            if(reason.empty()) {
                throw FailedTestException("Expected false, but got true.");
            } else {
                throw FailedTestException(reason);
            }
        }
    }
    
    static void assertTrue(bool value, std::string reason = "") {
        if(value == false) {
            if(reason.empty()) {
                throw FailedTestException("Expected true, but got false.");
            } else {
                throw FailedTestException(reason);
            }
        }
    }
    
};

#endif /* test_assertions_h */
