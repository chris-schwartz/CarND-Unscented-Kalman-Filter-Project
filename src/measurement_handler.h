//
//  measurement_handler.h
//  UnscentedKF
//
//  Created by Chris Schwartz on 5/31/17.
//
//



#ifndef measurement_handler_h
#define measurement_handler_h

#include <memory>

using namespace Eigen;
using namespace std;

class MeasurementHandler {
    
public:
    
    virtual ~MeasurementHandler() {
        
    }
    
    virtual unique_ptr<VectorXd> CreateInitialStateVector(MeasurementPackage measurement_package) {
        return NULL;
    }
    
    virtual void ProcessMeasurement(MeasurementPackage measurement_package) {
        return;
    }
    
    
    
};

#endif /* measurement_handler_h */
