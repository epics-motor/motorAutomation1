/*************************************************************************\
* Copyright (c) 2021 Aerotech, Inc.
* This file is distributed subject to a Software License Agreement found
* in the file LICENSE that is included with this distribution.
\*************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include "Automation1MotorController.h"
#include "Automation1MotorAxis.h"
#include "Include/Automation1.h"

/** Creates a new Automation1MotorController object.
  *
  * \param[in] portName           The name of the asyn port that will be created for this driver.
  * \param[in] hostName           The IP address or host name of the controller host machine.
  * \param[in] numAxes            The number of axes that this controller supports.
  * \param[in] movingPollPeriod   The time between polls when any axis is moving.
  * \param[in] idlePollPeriod     The time between polls when no axis is moving.
*/
Automation1MotorController::Automation1MotorController(const char* portName, const char* hostName, int numAxes, double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, numAxes, NUM_AUTOMATION1_PARAMS,
        0, // No additional interfaces beyond those in base class
        0, // No additional callback interfaces beyond those in base class
        ASYN_CANBLOCK | ASYN_MULTIDEVICE,
        1, // autoconnect
        0, 0)    // Default priority and stack size
{
    pAxes_ = (Automation1MotorAxis**)(asynMotorController::pAxes_);

    createAsynParams();

    // This function allocates a controller handle and gives a pointer to it.  It returns
    // true on success.
    if (!Automation1_ConnectWithHost(hostName, &(controller_)))
    {
        asynPrint(pasynUserSelf, 
                  ASYN_TRACE_ERROR,
            "Could not connect to Automation1 controller.\n");
        int errorCode = Automation1_GetLastError();
        char errorMessage[1024];
        char* pErrorMessage = errorMessage;
        Automation1_GetLastErrorMessage(pErrorMessage, 1024);
        asynPrint(pasynUserSelf, 
                  ASYN_TRACE_ERROR,
                  "Driver: Automation1. Function Message: Could not connect to Automation1 controller. API Error Code: %d\n. API Error Message: %s\n", 
                  errorCode, 
                  errorMessage);
    }
    bool isRunning = false;
    if (!Automation1_Controller_IsRunning(controller_, &isRunning))
    {
        asynPrint(pasynUserSelf, 
                  ASYN_TRACE_ERROR,
                  "Could not determine if Automation1 controller is running.\n");
    }
    // This command starts the controller if it is not already running.  
    // Note: If Program Automation is enabled, this means that programs 
    //       loaded on the controller may begin execution.
    if (!isRunning)
    {
        if (!Automation1_Controller_Start(controller_))
        {
            asynPrint(pasynUserSelf, 
                      ASYN_TRACE_ERROR,
                      "Could not start Automation1 controller.\n");
        }
    }

    for (int axis = 0; axis < numAxes; axis++)
    {
        // Note: We do not enable axes (i.e. setClosedLoop) at startup because 
        //       this involves actually powering the axis in question.  The 
        //       user should explicitly enable axes.
        new Automation1MotorAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

// Destructor.  Releases Automation1 handles.
Automation1MotorController::~Automation1MotorController()
{
    Automation1_DataCollectionConfig_Destroy(dataCollectionConfig_);
    Automation1_Disconnect(controller_);
}

void Automation1MotorController::createAsynParams(void) 
{
    createParam(AUTOMATION1_C_AckAllString,         asynParamInt32,     &AUTOMATION1_C_AckAll_);  //ajc-osl
}

/* * Creates a new Automation1 controller object.
* Configuration command, called directly or from iocsh.
    *
    * \param[in] portName            The name of the asyn port that will be created for this driver.
    * \param[in] hostName		     The IP address or host name of the controller host machine.
    * \param[in] numAxes             The number of axes that this controller supports.
    * \param[in] movingPollPeriod    The time in ms between polls when any axis is moving.
    * \param[in] idlePollPeriod      The time in ms between polls when no axis is moving.
  */
extern "C" int Automation1CreateController(const char* portName, const char* hostName, int numAxes, int movingPollPeriod, int idlePollPeriod)
{
    new Automation1MotorController(portName, hostName, numAxes, movingPollPeriod / 1000., idlePollPeriod / 1000.);
    return(asynSuccess);
}

/*  * Reports on status of the Automation1 controller.
    *
    * \param[in] fp The file pointer on which report information will be written
    * \param[in] level The level of report detail desired
    *
    * If details > 0 then information is printed about each axis.
    * After printing controller-specific information it calls asynMotorController::report()
*/
void Automation1MotorController::report(FILE* fp, int level)
{
    // Call the base class method
    asynMotorController::report(fp, level);
}

/*  * Returns a pointer to an Automation1 axis object.
    * Returns NULL if the axis number encoded in pasynUser is invalid.
    *
    * \param[in] pasynUser asynUser structure that encodes the axis index number.
*/
Automation1MotorAxis* Automation1MotorController::getAxis(asynUser* pasynUser)
{
    return static_cast<Automation1MotorAxis*>(asynMotorController::getAxis(pasynUser));
}


asynStatus Automation1MotorController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    bool status = true;
    if (function == AUTOMATION1_C_AckAll_) 
      {
	    //Call AckAll here
	    if(!Automation1_Command_AcknowledgeAll(controller_, 1))
	    {
	        logError("Could not clear faults.");
	    }
      }
	  
    //Call base class method. This will handle callCallbacks even if the function was handled here.
    status = (asynMotorController::writeInt32(pasynUser, value) == asynSuccess) && status;
    return asynSuccess;
}

/*  * Returns a pointer to an Automation1 axis object.
    * Returns NULL if the axis number is invalid.
    *
    * \param[in] axisNo Axis index number.
*/
Automation1MotorAxis* Automation1MotorController::getAxis(int axisNo)
{
    return static_cast<Automation1MotorAxis*>(asynMotorController::getAxis(axisNo));
}

asynStatus Automation1MotorController::initializeProfile(size_t maxProfilePoints)
{
    return asynMotorController::initializeProfile(maxProfilePoints);
}

asynStatus Automation1MotorController::buildProfile()
{
    int i, j;
    bool buildOK = true;
    int numPoints;
    int numPulses;
    int timeMode;
    double profileTime;
    double pulseTime;
    int buildStatus;
    int moveMode;
    int numUsedAxes;
    double resolution;
    int useAxis;
    std::string profileMoveFileContents;

    setIntegerParam(profileBuildState_, PROFILE_BUILD_BUSY);
    setIntegerParam(profileBuildStatus_, PROFILE_STATUS_UNDEFINED);
    getIntegerParam(profileNumPoints_, &numPoints);
    getIntegerParam(profileNumPulses_, &numPulses);
    getIntegerParam(profileTimeMode_, &timeMode);
    getIntegerParam(profileMoveMode_, &moveMode);
    callParamCallbacks();

    // Call the base class method to initialize the time array.
    asynMotorController::buildProfile();

    // To avoid needless iteration, we make a vector of the axes in use during profile motion.
    // We also query each axis's resolution as it is needed for multiple operations.
    profileAxes_.clear();
    profileAxesResolutions_.clear();
    profileAxes_.reserve(numAxes_);
    profileAxesResolutions_.reserve(numAxes_);
    for (i = 0; i < numAxes_; i++)
    {
        getIntegerParam(i, profileUseAxis_, &useAxis);
        if (useAxis)
        {
            profileAxes_.push_back(i);
            getDoubleParam(i, motorRecResolution_, &resolution);
            profileAxesResolutions_.push_back(resolution);
        }
    }
    profileAxes_.shrink_to_fit();
    profileAxesResolutions_.shrink_to_fit();
    numUsedAxes = profileAxes_.size();

    // For the initial release, it was decided to limit profile motion to 1 ms increments in order
    // to keep motion in-sync with the real time data collection.
    if (timeMode != PROFILE_TIME_MODE_FIXED)
    {
        buildOK = false;
        asynPrint(pasynUserSelf, 
                  ASYN_TRACE_ERROR, 
                  "Currently the Automation1 EPICS offering only supports fixed-time, 1 ms profile points. Time mode is not currently set to fixed.");
        goto done;
    }
    
    for (i = 0; i < numPoints; i++)
    {
        if (profileTimes_[i] != 0.001)
        {
            buildOK = false;
            asynPrint(pasynUserSelf, 
                      ASYN_TRACE_ERROR, 
                      "Currently the Automation1 EPICS offering only supports fixed-time, 1 ms profile points. Index %d of the time array has a time of %f", 
                      i, 
                      profileTimes_[i]);
            goto done;
        }
    }

    // We need to make a configuration handle for the data points we want to log at each pulse.  
    // This will be used to retrieve data from the controller.
    if (!dataCollectionConfig_)
    {
        if (!Automation1_DataCollectionConfig_Create(Automation1DataCollectionFrequency_1kHz, numPulses, &dataCollectionConfig_))
        {
            buildOK = false;
            logError("Could not create dataCollectionConfig.");
            goto done;
        }
    }

    // Clear all signals that may be present in the configuration.
    if (!Automation1_DataCollectionConfig_ClearAllDataSignals(dataCollectionConfig_))
    {
        buildOK = false;
        logError("Could not clear dataCollectionConfig.");
        goto done;
    }

    // We must add the required signals to the configuration handle.
    for (i = 0; i < numUsedAxes; i++)
    {
        if (!Automation1_DataCollectionConfig_AddAxisDataSignal(dataCollectionConfig_,
                                                                profileAxes_[i],
                                                                Automation1AxisDataSignal_ProgramPositionFeedback,
                                                                0))
        {
            buildOK = false;
            logError("Could not add axis signal to dataCollectionConfig.");
            goto done;
        }

        if (!Automation1_DataCollectionConfig_AddAxisDataSignal(dataCollectionConfig_,
                                                                profileAxes_[i],
                                                                Automation1AxisDataSignal_PositionError,
                                                                0))
        {
            buildOK = false;
            logError("Could not add axis signal to dataCollectionConfig.");
            goto done;
        }
    }

    // Currently the only way to guarantee timing using the C API is to upload a file to the
    // controller and execute it on a separate task.  This string is used to assemble the file
    // contents in memory.
    //
    // Note: Preallocation is needed to avoid memory issues that can occur when appending
    //       large amounts of content to a string. 128 might be overkill, so if there are
    //       issues with memory usage later on, this value should be changed.
    profileMoveFileContents.reserve(numPoints * 128);

    profileMoveFileContents.append("program\n");

    // This constructs a string that will become an array of ints that represent the axes
    // on which we want to operate in the Aeroscript program.
    profileMoveFileContents.append("var $axes[] as axis = [");
    for (i = 0; i < numUsedAxes; i++)
    {
        profileMoveFileContents.append("@");
        profileMoveFileContents.append(std::to_string(profileAxes_[i]));
        if (i != numUsedAxes - 1)
        {
            profileMoveFileContents.append(",");
        }
        else
        {
            profileMoveFileContents.append("]\n");
        }
    }

    // In order to run Pt moves, the task that will run the moves must have "3-position / 1-velocity"
    // interpolation mode (task value 1).  By default, the task is in "2-position / 2-velocity"
    // interpolation mode (task value 0).
    //
    // Note: Task 2 was chosen somewhat arbitrarily.  Task 0 is reserved by the controller, but other
    //       tasks can be used for user commands. 
    profileMoveFileContents.append("var $motionInterpolationMode as real = ParameterGetTaskValue(");
    profileMoveFileContents.append(std::to_string(PROFILE_MOVE_TASK_INDEX));

    profileMoveFileContents.append(", TaskParameter.MotionInterpolationMode)\n");
    profileMoveFileContents.append("ParameterSetTaskValue(");
    profileMoveFileContents.append(std::to_string(PROFILE_MOVE_TASK_INDEX));
    profileMoveFileContents.append(", TaskParameter.MotionInterpolationMode, 1)\n");

    // We set the task mode in accordance with what the user specified.
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
    {
        profileMoveFileContents.append("SetupTaskTargetMode(TargetMode.Absolute)\n");
    }
    else
    {
        profileMoveFileContents.append("SetupTaskTargetMode(TargetMode.Incremental)\n");
    }

    // We start data collection just before the actual profile moves.
    profileMoveFileContents.append("AppDataCollectionSnapshot()\n");

    // This block assembled the main part of the program.  Each profile time corresponds to one
    // movePtCommand in Aereoscript.
    for (i = 0; i < numPoints; i++)
    {
        profileMoveFileContents.append("MovePt($axes, [");
        for (j = 0; j < numUsedAxes; j++)
        {
            profileMoveFileContents.append(std::to_string(pAxes_[profileAxes_[j]]->profilePositions_[i] * profileAxesResolutions_[j]));
            if (j != numUsedAxes - 1)
            {
                profileMoveFileContents.append(",");
            }
            else
            {
                profileMoveFileContents.append("], ");
            }
        }
        // Automation1 assumes that this value is in ms, not s, so we perform the conversion.
        profileMoveFileContents.append(std::to_string(profileTimes_[i] * 1000));
        profileMoveFileContents.append(")\n");
    }

    profileMoveFileContents.append("WaitForMotionDone($axes)\n");
    profileMoveFileContents.append("AppDataCollectionStop()\n");
    profileMoveFileContents.append("ParameterSetTaskValue(");
    profileMoveFileContents.append(std::to_string(PROFILE_MOVE_TASK_INDEX));
    profileMoveFileContents.append(", TaskParameter.MotionInterpolationMode, $motionInterpolationMode)\n");
    profileMoveFileContents.append("end");

    // This command writes the file to the controller.  Note that if the profile move file
    // already exists, it will be overwritten.
    if (!Automation1_Files_WriteBytes(controller_,
        "epics_profile_move.ascript",
        reinterpret_cast<const uint8_t*>(profileMoveFileContents.data()),
        profileMoveFileContents.size()))
    {
        buildOK = false;
        logError("Could not write profile file to controller.");
        goto done;
    }

done:

    buildStatus = buildOK ? PROFILE_STATUS_SUCCESS : PROFILE_STATUS_FAILURE;
    setIntegerParam(profileBuildStatus_, buildStatus);
    if (buildStatus != PROFILE_STATUS_SUCCESS) {
        asynPrint(pasynUserSelf, 
                  ASYN_TRACE_ERROR,
                  "profile build failed\n");
    }
    // Clear build command.  This is a "busy" record, don't want to do this until build is complete.
    setIntegerParam(profileBuild_, 0);
    setIntegerParam(profileBuildState_, PROFILE_BUILD_DONE);
    callParamCallbacks();
    return buildOK ? asynSuccess : asynError;
}

asynStatus Automation1MotorController::executeProfile()
{
    bool executeOK = true;
    int executeStatus;

    setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_MOVE_START);
    setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_UNDEFINED);
    callParamCallbacks();
    setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_EXECUTING);

    // This is a bit of a workaround to make the Automation1 controller record the correct data signals.
    // No data will actually be recorded here.
    Automation1_DataCollection_Start(controller_, 
                                     dataCollectionConfig_,
                                     Automation1DataCollectionMode_Snapshot);
    Automation1_DataCollection_Stop(controller_);

    // This compiles and runs the Aeroscript file on the controller. Note that this function returns
    // after the program is compiled and started, it does not wait for the program to finish.
    if (!Automation1_Task_ProgramRun(controller_,
                                     PROFILE_MOVE_TASK_INDEX,
                                     "epics_profile_move.ascript"))
    {
        executeOK = false;
        logError("Failed to compile and run profile move file.");
    }

done:

    // Check for task 2 status, program running, program complete (task status enum).
    if (executeOK)
    {
        executeStatus = PROFILE_STATUS_SUCCESS;
    }
    else
    {
        executeStatus = PROFILE_STATUS_FAILURE;
    }
    executeStatus = executeOK ? PROFILE_STATUS_SUCCESS : PROFILE_STATUS_FAILURE;
    setIntegerParam(profileExecuteStatus_, executeStatus);
    setIntegerParam(profileExecute_, 0);
    // Note: This state is not exactly accurate.  We may need to run this function on 
    //       a separate EPICS thread and check the Automation1 task state to see if 
    //       motion has been completed.
    setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_DONE);
    callParamCallbacks();

    return executeOK ? asynSuccess : asynError;
}

asynStatus Automation1MotorController::abortProfile()
{
    if (!Automation1_Task_ProgramStop(controller_,
                                      PROFILE_MOVE_TASK_INDEX,
                                      PROFILE_MOVE_ABORT_TIMEOUT))
    {
        logError("Automation1 profile move failed to abort.");
        return asynError;
    }
    return asynSuccess;
}

asynStatus Automation1MotorController::readbackProfile()
{
    int i, j;
    int axis;
    bool readbackOK = true;
    int readbackStatus;
    Automation1DataCollectionStatus dataCollectionStatus;
    int numProfileAxes;
    int allResultsLength;
    int allResultsSize;
    double* allResults;
    int readPoints;
    int signalResultsSize;
    double* signalResults;

    setIntegerParam(profileReadbackState_, PROFILE_READBACK_BUSY);
    setIntegerParam(profileReadbackStatus_, PROFILE_STATUS_UNDEFINED);
    callParamCallbacks();

    // Continuously poll the controller until data collection is done.
    Automation1_DataCollection_GetStatus(controller_, &dataCollectionStatus);
    while (dataCollectionStatus.IsCollecting)
    {
        if (!Automation1_DataCollection_GetStatus(controller_, &dataCollectionStatus))
        {
            readbackOK = false;
            logError("Failed to get data collection status.");
            goto done;
        }
    }

    readPoints = dataCollectionStatus.NumberOfRetrievedPoints;
    numProfileAxes = profileAxes_.size();
    allResultsLength = readPoints * profileAxes_.size() * 2;
    allResultsSize = allResultsLength * sizeof(double);
    allResults = (double*)malloc(allResultsSize);

    if (readPoints > maxProfilePoints_)
    {
        readPoints = maxProfilePoints_;
    }
    signalResultsSize = readPoints * sizeof(double);
    signalResults = (double*)malloc(signalResultsSize);

    for (i = 0; i < numAxes_; i++)
    {
        memset(pAxes_[i]->profileReadbacks_, 0, maxProfilePoints_ * sizeof(double));
        memset(pAxes_[i]->profileFollowingErrors_, 0, maxProfilePoints_ * sizeof(double));
    }

    if (!Automation1_DataCollection_GetResults(controller_,
                                               dataCollectionConfig_,
                                               allResults,
                                               allResultsLength))
    {
        readbackOK = false;
        logError("Failed to get data collection results.");
        goto done;
    }

    for (i = 0; i < numProfileAxes; i++)
    {
        axis = profileAxes_[i];
        if (!Automation1_DataCollection_GetAxisResults(dataCollectionConfig_, 
                                                       allResults, 
                                                       allResultsLength, 
                                                       axis, 
                                                       Automation1AxisDataSignal_ProgramPositionFeedback, 
                                                       0, 
                                                       signalResults, 
                                                       readPoints))
        {
            readbackOK = false;
            logError("Failed to parse program position feedback results.");
            goto done;
        }
        // Need to convert the readback into steps for the record.
        for (j = 0; j < readPoints; j++)
        {
            signalResults[j] = signalResults[j] / profileAxesResolutions_[i];
        }
        memcpy(pAxes_[i]->profileReadbacks_, signalResults, signalResultsSize);

        if (!Automation1_DataCollection_GetAxisResults(dataCollectionConfig_, 
                                                       allResults, 
                                                       allResultsLength, 
                                                       axis, 
                                                       Automation1AxisDataSignal_PositionError, 
                                                       0, 
                                                       signalResults, 
                                                       readPoints))
        {
            readbackOK = false;
            logError("Failed to parse position error results.");
            goto done;
        }
        for (j = 0; j < readPoints; j++)
        {
            signalResults[j] = signalResults[j] / profileAxesResolutions_[i];
        }
        memcpy(pAxes_[i]->profileFollowingErrors_, signalResults, signalResultsSize);
    }

done:

    free(allResults);
    free(signalResults);
    setIntegerParam(profileNumReadbacks_, readPoints);

    for (i = 0; i < numProfileAxes; i++)
    {
        pAxes_[i]->readbackProfile();
    }

    // Clear readback command.  This is a "busy" record, don't want to do this until readback is complete.
    readbackStatus = readbackOK ? PROFILE_STATUS_SUCCESS : PROFILE_STATUS_FAILURE;
    setIntegerParam(profileReadbackStatus_, readbackStatus);
    setIntegerParam(profileReadback_, 0);
    setIntegerParam(profileReadbackState_, PROFILE_READBACK_DONE);
    callParamCallbacks();

    return readbackOK ? asynSuccess : asynError;
}

/** Logs an driver error and error details from the C API.  Made to reduce duplicate code.
  * \param[in] driverMessage A char array meant to convey where in execution the error occured.
*/
void Automation1MotorController::logError(const char* driverMessage)
{
    int errorCode = Automation1_GetLastError();
    char errorMessage[1024];
    Automation1_GetLastErrorMessage(errorMessage, 1024);

    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "Driver: Automation1. Function Message: %s. API Error Code: %d. API Error Message: %s\n",
        driverMessage,
        errorCode,
        errorMessage);

    return;
}


asynStatus Automation1CreateProfile(const char *portName, int maxPoints)
{
    Automation1MotorController *pC;
    static const char *functionName = "Automation1CreateProfile";

    pC = (Automation1MotorController*) findAsynPortDriver(portName);
    if (!pC)
    {
        printf("Automation1:%s: Error port %s not found\n",
               functionName, portName);
        return asynError;
    }
    pC->initializeProfile(maxPoints);
    return asynSuccess;
}

// Profile Setup arguments
static const iocshArg Automation1CreateProfileArg0 = {"Port name", iocshArgString};
static const iocshArg Automation1CreateProfileArg1 = {"Max points", iocshArgInt};

static const iocshArg* const Automation1CreateProfileArgs[2] = {&Automation1CreateProfileArg0, &Automation1CreateProfileArg1};

static const iocshFuncDef configAutomation1Profile = {"Automation1CreateProfile", 2, Automation1CreateProfileArgs};

static void configAutomation1ProfileCallFunc(const iocshArgBuf* args)
{
    Automation1CreateProfile(args[0].sval, args[1].ival);
}

// Code for iocsh registration
static const iocshArg Automation1CreateControllerArg0 = { "Port name", iocshArgString };
static const iocshArg Automation1CreateControllerArg1 = { "Host name", iocshArgString };
static const iocshArg Automation1CreateControllerArg2 = { "Number of axes", iocshArgInt };
static const iocshArg Automation1CreateControllerArg3 = { "Moving poll period (ms)", iocshArgInt };
static const iocshArg Automation1CreateControllerArg4 = { "Idle poll period (ms)", iocshArgInt };
static const iocshArg* const Automation1CreateControllerArgs[] = { &Automation1CreateControllerArg0,
                                                                   &Automation1CreateControllerArg1,
                                                                   &Automation1CreateControllerArg2,
                                                                   &Automation1CreateControllerArg3,
                                                                   &Automation1CreateControllerArg4 };
static const iocshFuncDef Automation1CreateControllerDef = { "Automation1CreateController", 5, Automation1CreateControllerArgs };
static void Automation1CreateContollerCallFunc(const iocshArgBuf* args)
{
    Automation1CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void Automation1Register(void)
{
    iocshRegister(&Automation1CreateControllerDef, Automation1CreateContollerCallFunc);
    iocshRegister(&configAutomation1Profile, configAutomation1ProfileCallFunc);
}

extern "C" {
    epicsExportRegistrar(Automation1Register);
}
