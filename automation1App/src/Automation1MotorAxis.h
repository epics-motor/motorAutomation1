/*************************************************************************\
* Copyright (c) 2021 Aerotech, Inc.
* This file is distributed subject to a Software License Agreement found
* in the file LICENSE that is included with this distribution.
\*************************************************************************/

#ifndef Automation1MotorAxis_H
#define Automation1MotorAxis_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "Include/Automation1.h"

class Automation1MotorController;

class epicsShareClass Automation1MotorAxis : public asynMotorAxis
{
public:
    // Member functions we override from the base class.
    Automation1MotorAxis(Automation1MotorController* pC, int axis);
    ~Automation1MotorAxis();
    void report(FILE* fp, int level);
    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus poll(bool* moving);
    asynStatus setPosition(double position);
    asynStatus setClosedLoop(bool closedLoop);

    // Needed for profile motion.
    asynStatus defineProfile(double* positions, size_t numPoints);
    asynStatus readbackProfile();

private:
    // Pointer to asynMotorController to which the axis belongs.
    Automation1MotorController* pC_;

    // The config is used by certain functions in the Automation1 C API to 
    // get status items from the controller.  This is needed for polling.
    Automation1StatusConfig statusConfig_;

    // Automation1 error codes and messages must be acquired through
    // calls to the C API.  To avoid duplicate code, we wrap calls 
    // to those functions and to asynPrint in this function.
    void logError(const char* driverMessage);
    
    double countsPerUnitParam_;    
    
    friend class Automation1MotorController;
};

#endif
