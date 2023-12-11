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
#define PROFILE_MOVE_ABORT_TIMEOUT 1000
#define DATA_POINTS_PER_SECOND 1000

// New params added
#define AUTOMATION1_C_AckAllString          "AUTOMATION1_C_ACKALL"	//ajc-osl
#define AUTOMATION1_C_VelocityString        "AUTOMATION1_C_VELOCITY"
#define AUTOMATION1_C_FErrorString          "AUTOMATION1_C_FERROR"
#define AUTOMATION1_C_ExecuteCommandString  "AUTOMATION1_C_EXECUTE_COMMAND"
// Controller-specific profileMove parameters
#define AUTOMATION1_PM_PulseModeString      "AUTOMATION1_PM_PULSE_MODE"
#define AUTOMATION1_PM_PulsePosString       "AUTOMATION1_PM_PULSE_POS"
#define AUTOMATION1_PM_NumPulsesString      "AUTOMATION1_PM_NUM_PULSES"
#define AUTOMATION1_PM_PulseDirString       "AUTOMATION1_PM_PULSE_DIR"
#define AUTOMATION1_PM_PulseLenString       "AUTOMATION1_PM_PULSE_LEN"
#define AUTOMATION1_PM_PulsePeriodString    "AUTOMATION1_PM_PULSE_PERIOD"
#define AUTOMATION1_PM_PulseSrcString       "AUTOMATION1_PM_PULSE_SRC"
#define AUTOMATION1_PM_PulseOutString       "AUTOMATION1_PM_PULSE_OUT"
#define AUTOMATION1_PM_PulseAxisString      "AUTOMATION1_PM_PULSE_AXIS"
#define NUM_AUTOMATION1_PARAMS 11


class epicsShareClass Automation1MotorController : public asynMotorController
{
public:
    // Member functions we override from the base class.
    Automation1MotorController(const char* portName, const char* hostName, int numAxes, double movingPollPeriod, double idlePollPeriod, int commandExecuteTask, int profileMoveTask);
    ~Automation1MotorController();
    void report(FILE* fp, int level);
    Automation1MotorAxis* getAxis(asynUser* pasynUser);
    Automation1MotorAxis* getAxis(int axisNo);

    /* These are the methods that we override */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);	//ajc-osl
    asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
    void createAsynParams(void);

    // These are functions for profile moves.
    asynStatus initializeProfile(size_t maxProfilePoints, size_t maxProfilePulses);
    asynStatus definePulses(int pulseAxis, size_t numPulses);
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
    int AUTOMATION1_PM_PulseMode_;
    int AUTOMATION1_PM_PulsePos_;
    int AUTOMATION1_PM_NumPulses_;
    int AUTOMATION1_PM_PulseDir_;
    int AUTOMATION1_PM_PulseLen_;
    int AUTOMATION1_PM_PulsePeriod_;
    int AUTOMATION1_PM_PulseSrc_;
    int AUTOMATION1_PM_PulseOut_;
    int AUTOMATION1_PM_PulseAxis_;
    int parameters[NUM_AUTOMATION1_PARAMS];

private:

    // An Automation1 Controller Handle used by the C API to
    // actually execute commands on the controller.
    Automation1Controller controller_;

    // A handle that will be used to specify the data logged for
    // readbacks.
    Automation1DataCollectionConfig dataCollectionConfig_;
    
    // User-specified task indices
    int32_t commandExecuteTask_;
    int32_t profileMoveTask_;
    
    // The total number of data collection points
    int numDataPoints_;
    // The ratio of recorded data points to profile waypoints
    int displayPointSpacing_;
    //
    int numPulses_;
    size_t maxProfilePulses_;
    double *profilePulses_;
    double *profilePulsesUser_;
    double *profilePulseDisplacements_;
    int32_t profilePulseDisplacementsIndex_;
    int fullProfileSize_;
    double *fullProfileTimes_;
    int32_t fullProfileTimesIndex_;
    int32_t globalVarOffset_;
    
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
