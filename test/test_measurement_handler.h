//
// Created by Chris Schwartz on 6/4/17.
//

#ifndef UNSCENTEDKF_TEST_MEASUREMENT_HANDLER_H
#define UNSCENTEDKF_TEST_MEASUREMENT_HANDLER_H

#include "test_case.h"
class TestMeasurementHandler : public TestCase<TestMeasurementHandler> {
 public:
    TestMeasurementHandler();

    virtual ~TestMeasurementHandler();

    void testUpdate();

};

#endif //UNSCENTEDKF_TEST_MEASUREMENT_HANDLER_H
