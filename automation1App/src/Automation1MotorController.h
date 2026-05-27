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
#define MAX_AUTOMATION1_HEXAPODS 2
#define HEXAPOD_NUM_AXES 6
#define PROFILE_MOVE_ABORT_TIMEOUT 1000
#define DATA_POINTS_PER_SECOND 1000

// New params added
#define AUTOMATION1_C_AckAllString          "AUTOMATION1_C_ACKALL"	//ajc-osl
#define AUTOMATION1_C_VelocityString        "AUTOMATION1_C_VELOCITY"
#define AUTOMATION1_C_FErrorString          "AUTOMATION1_C_FERROR"
#define AUTOMATION1_C_ExecuteCommandString  "AUTOMATION1_C_EXECUTE_COMMAND"
#define AUTOMATION1_C_AxisStatusBits        "AUTOMATION1_C_AXIS_STATUS_BITS"
#define AUTOMATION1_C_DriveStatusBits       "AUTOMATION1_C_DRIVE_STATUS_BITS"
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
// Hexapod readback parameters
#define AUTOMATION1_HXP_StateString         "AUTOMATION1_HXP_STATE"
#define AUTOMATION1_HXP_ReadModeString      "AUTOMATION1_HXP_READ_MODE"
#define AUTOMATION1_HXP_WriteModeString     "AUTOMATION1_HXP_WRITE_MODE"
// Hexapod coordinated-move parameters
#define AUTOMATION1_HXP_MoveAllString       "AUTOMATION1_HXP_MOVE_ALL"
#define AUTOMATION1_HXP_TargetXString       "AUTOMATION1_HXP_TARGET_X"
#define AUTOMATION1_HXP_TargetYString       "AUTOMATION1_HXP_TARGET_Y"
#define AUTOMATION1_HXP_TargetZString       "AUTOMATION1_HXP_TARGET_Z"
#define AUTOMATION1_HXP_TargetAString       "AUTOMATION1_HXP_TARGET_A"
#define AUTOMATION1_HXP_TargetBString       "AUTOMATION1_HXP_TARGET_B"
#define AUTOMATION1_HXP_TargetCString       "AUTOMATION1_HXP_TARGET_C"
#define AUTOMATION1_HXP_VelocityString      "AUTOMATION1_HXP_VELOCITY"
#define NUM_AUTOMATION1_PARAMS 26


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

    // Hexapod initialization
    asynStatus initializeHexapod(int hexapodIndex, int firstHexapodAxis);

    // Hexapod state/mode readback (called from poll)
    asynStatus getHexapodState(int hexapodIndex);
    asynStatus getHexapodMode(int hexapodIndex);

    // Hexapod mode setpoint (called from writeInt32)
    asynStatus setHexapodMode(int hexapodIndex, int mode);

    // Hexapod coordinated move (called from writeInt32 on MoveAll rising edge)
    asynStatus hexapodMoveAll(int hexapodIndex);

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
    int AUTOMATION1_C_AxisStatusBits_;
    int AUTOMATION1_C_DriveStatusBits_;
    int AUTOMATION1_PM_PulseMode_;
    int AUTOMATION1_PM_PulsePos_;
    int AUTOMATION1_PM_NumPulses_;
    int AUTOMATION1_PM_PulseDir_;
    int AUTOMATION1_PM_PulseLen_;
    int AUTOMATION1_PM_PulsePeriod_;
    int AUTOMATION1_PM_PulseSrc_;
    int AUTOMATION1_PM_PulseOut_;
    int AUTOMATION1_PM_PulseAxis_;
    int AUTOMATION1_HXP_State_;
    int AUTOMATION1_HXP_ReadMode_;
    int AUTOMATION1_HXP_WriteMode_;
    int AUTOMATION1_HXP_MoveAll_;
    int AUTOMATION1_HXP_TargetX_;
    int AUTOMATION1_HXP_TargetY_;
    int AUTOMATION1_HXP_TargetZ_;
    int AUTOMATION1_HXP_TargetA_;
    int AUTOMATION1_HXP_TargetB_;
    int AUTOMATION1_HXP_TargetC_;
    int AUTOMATION1_HXP_Velocity_;
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

    // Hexapod tracking state
    int numHexapods_;
    int firstHexapodAxisIndex_[MAX_AUTOMATION1_HEXAPODS];

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

    // Execute an AeroScript integer-returning expression on commandExecuteTask_.
    // `expression` is the right-hand side (e.g. "GetHexapodState(0)"); the method
    // prepends "$ireturn[0]=" before sending. On success, *valueOut receives the
    // returned integer.
    asynStatus writeReadInt(const char *expression, int64_t *valueOut);

    friend class Automation1MotorAxis;
};

#endif
