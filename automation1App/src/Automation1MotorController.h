/*************************************************************************\
* Copyright (c) 2021 Aerotech, Inc.
* This file is distributed subject to a Software License Agreement found
* in the file LICENSE that is included with this distribution.
\*************************************************************************/

#ifndef Automation1MotorController_H
#define Automation1MotorController_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "Automation1MotorAxis.h"
#include "Include/Automation1.h"

#include <cstdarg>

#define MAX_AUTOMATION1_AXES 32
#define PROFILE_MOVE_TASK_INDEX 2
#define PROFILE_MOVE_ABORT_TIMEOUT 1000
#define DATA_POINTS_PER_SECOND 1000

// New params added
#define AUTOMATION1_C_AckAllString          "AUTOMATION1_C_ACKALL"	//ajc-osl
#define AUTOMATION1_C_VelocityString        "AUTOMATION1_C_VELOCITY"
#define AUTOMATION1_C_FErrorString          "AUTOMATION1_C_FERROR"
#define AUTOMATION1_C_ExecuteCommandString  "AUTOMATION1_C_EXECUTE_COMMAND"
#define NUM_AUTOMATION1_PARAMS 4


class epicsShareClass Automation1MotorController : public asynMotorController
{
public:
    // Member functions we override from the base class.
    Automation1MotorController(const char* portName, const char* hostName, int numAxes, double movingPollPeriod, double idlePollPeriod);
    ~Automation1MotorController();
    void report(FILE* fp, int level);
    Automation1MotorAxis* getAxis(asynUser* pasynUser);
    Automation1MotorAxis* getAxis(int axisNo);

    /* These are the methods that we override */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);	//ajc-osl
    asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    void createAsynParams(void);

    // These are functions for profile moves.
    asynStatus initializeProfile(size_t maxProfilePoints);
    asynStatus buildProfile();
    asynStatus executeProfile();
    asynStatus abortProfile();
    asynStatus readbackProfile();

    asynStatus poll() override;
protected:

    // Array of pointers to axis objects.
    Automation1MotorAxis** pAxes_;

    int AUTOMATION1_C_AckAll_;
    int AUTOMATION1_C_Velocity_;
    int AUTOMATION1_C_FError_;
    int AUTOMATION1_C_ExecuteCommand_;
    int parameters[NUM_AUTOMATION1_PARAMS];

private:

    // An Automation1 Controller Handle used by the C API to
    // actually execute commands on the controller.
    Automation1Controller controller_;

    // A handle that will be used to specify the data logged for
    // readbacks.
    Automation1DataCollectionConfig dataCollectionConfig_;

    // Axes to be used in a profile move.
    std::vector<int> profileAxes_;

    // Convience wrapper for reporting Automation1 API errors
    void logApiError(const char* driverMessage) {
        logApiError(-1, driverMessage);
    }
    void logApiError(int messageIndex, const char* driverMessage);

    void logError(const char* fmt, ...) {
        std::va_list args;
        va_start(args, fmt);
        logErrorV(-1, fmt, args);
        va_end(args);
    }

    void logError(int messageIndex, const char* fmt, ...) {
        std::va_list args;
        va_start(args, fmt);
        logErrorV(messageIndex, fmt, args);
        va_end(args);
    }

    // Log an error with asyn and post a message PV if messageIndex != -1
    void logErrorV(int messageIndex, const char* fmt, std::va_list agrs);

    friend class Automation1MotorAxis;
};

#endif
