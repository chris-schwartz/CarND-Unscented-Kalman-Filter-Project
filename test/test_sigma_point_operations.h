//
//  test_sigma_point_operations.h
//  UnscentedKF
//
//  Created by Chris Schwartz on 6/2/17.
//
//


#pragma once

#include "test_case.h"
#include "sigma_point_operations.h"
#include "eigen_assert.h"

class TestSigmaPointOperations : public TestCase<TestSigmaPointOperations> {
    
public:

    TestSigmaPointOperations();

    ~TestSigmaPointOperations();

    void testGeneratingSigmaPoints();

    void testPredictingSigmaPoints();

    void testPredictingMean();

    void testPredictingCovariance();

};

