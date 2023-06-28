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

static const char *driverName = "Automation1MotorController";

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
    dataCollectionConfig_ = NULL;
    displayPointSpacing_ = 1;
    pAxes_ = (Automation1MotorAxis**)(asynMotorController::pAxes_);

    createAsynParams();

    // This function allocates a controller handle and gives a pointer to it.  It returns
    // true on success.
    if (!Automation1_ConnectWithHost(hostName, &(controller_)))
    {
        logApiError("Could not connect to Automation1 controller");
    }
    bool isRunning = false;
    if (!Automation1_Controller_IsRunning(controller_, &isRunning))
    {
        logApiError("Could not determine if Automation1 controller is running");
    }
    // This command starts the controller if it is not already running.  
    // Note: If Program Automation is enabled, this means that programs 
    //       loaded on the controller may begin execution.
    if (!isRunning)
    {
        if (!Automation1_Controller_Start(controller_))
        {
            logApiError("Could not start Automation1 controller");
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
    createParam(AUTOMATION1_C_VelocityString,       asynParamFloat64,   &AUTOMATION1_C_Velocity_);
    createParam(AUTOMATION1_C_FErrorString,         asynParamFloat64,   &AUTOMATION1_C_FError_);
    createParam(AUTOMATION1_C_ExecuteCommandString, asynParamOctet,     &AUTOMATION1_C_ExecuteCommand_);
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


asynStatus Automation1MotorController::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual)
{
    int function = pasynUser->reason;
    
    if(function == AUTOMATION1_C_ExecuteCommand_)
    {
        if(!Automation1_Command_Execute(this->controller_, 1, value))
        {
            logError("Could not execute requested command.");
        }
    }

    // Call base method.
    asynPortDriver::writeOctet(pasynUser, value, maxChars, nActual);
    return asynSuccess;
}

asynStatus Automation1MotorController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    bool status = true;
    
    if (function == AUTOMATION1_C_AckAll_) 
    {        
	    if(!Automation1_Command_AcknowledgeAll(controller_, 1))
	    {
	        logError("Could not clear faults.");
	    }
	 
	    for (int i = 0; i < numAxes_; i++)
        {
	        (getAxis(i))->setIntegerParam(this->motorStatusProblem_,0);            // Unset "Problem" status bit on all axes.
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
    int buildStatus;
    int moveMode;
    int numUsedAxes;
    int useAxis;
    double timePerPoint;
    double totalTime;
    std::string profileMoveFileContents;
    Automation1MotorAxis* axis;
    static const char *functionName="buildProfile";

    setIntegerParam(profileBuildState_, PROFILE_BUILD_BUSY);
    setIntegerParam(profileBuildStatus_, PROFILE_STATUS_UNDEFINED);
    setStringParam(profileBuildMessage_, "");
    getIntegerParam(profileNumPoints_, &numPoints);
    getIntegerParam(profileNumPulses_, &numPulses);
    getIntegerParam(profileTimeMode_, &timeMode);
    getDoubleParam(profileFixedTime_, &timePerPoint);
    getIntegerParam(profileMoveMode_, &moveMode);
    callParamCallbacks();

    // Call the base class method to initialize the time array.
    asynMotorController::buildProfile();

    // To avoid needless iteration, we make a vector of the axes in use during profile motion.
    // We also query each axis's resolution as it is needed for multiple operations.
    profileAxes_.clear();
    profileAxes_.reserve(numAxes_);
    for (i = 0; i < numAxes_; i++)
    {
        getIntegerParam(i, profileUseAxis_, &useAxis);
        if (useAxis)
        {
            profileAxes_.push_back(i);
        }
    }
    profileAxes_.shrink_to_fit();
    numUsedAxes = profileAxes_.size();

    // The data collection points will be based on the total trajectory time and the 1kHz collection frequency
    // The profile trajectory will usually have many fewer points in it.
    if (timeMode == PROFILE_TIME_MODE_FIXED)
    {
        totalTime = timePerPoint * numPoints;
    }
    else
    {
        totalTime = 0;
        for (i=0; i < numPoints; i++)
        {
            totalTime += profileTimes_[i];
        }
    }
    // The lowest data collection frequency is currently 1kHz. Arbitrary frequencies will likely be supported in the future.
    numDataPoints_ = totalTime * 1000;
    displayPointSpacing_ = numDataPoints_ / numPoints;
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: totalTime = %lf, numDataPoints = %i, displayPointSpacing_ = %i\n", driverName, functionName, totalTime, numDataPoints_, displayPointSpacing_);
    
    // We need to make a configuration handle for the data points we want to collect.  
    // This will be used to retrieve data from the controller.
    if (!dataCollectionConfig_)
    {
        if (!Automation1_DataCollectionConfig_Create(Automation1DataCollectionFrequency_1kHz, numDataPoints_, &dataCollectionConfig_))
        {
            buildOK = false;
            logApiError(profileBuildMessage_, "Could not create dataCollectionConfig");
            goto done;
        }
    }

    // Clear all signals that may be present in the configuration.
    if (!Automation1_DataCollectionConfig_ClearAllDataSignals(dataCollectionConfig_))
    {
        buildOK = false;
        logApiError(profileBuildMessage_, "Could not clear dataCollectionConfig");
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
            logApiError(profileBuildMessage_, "Could not add axis signal to dataCollectionConfig.");
            goto done;
        }

        if (!Automation1_DataCollectionConfig_AddAxisDataSignal(dataCollectionConfig_,
                                                                profileAxes_[i],
                                                                Automation1AxisDataSignal_PositionError,
                                                                0))
        {
            buildOK = false;
            logApiError(profileBuildMessage_, "Could not add axis signal to dataCollectionConfig.");
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
            axis = pAxes_[profileAxes_[j]];
            profileMoveFileContents.append(std::to_string(axis->profilePositions_[i] / axis->countsPerUnitParam_));
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
        logApiError(profileBuildMessage_, "Could not write profile file to controller");
        goto done;
    }

    setIntegerParam(profileCurrentPoint_, 0);

done:

    buildStatus = buildOK ? PROFILE_STATUS_SUCCESS : PROFILE_STATUS_FAILURE;
    setIntegerParam(profileBuildStatus_, buildStatus);

    // Clear build command.  This is a "busy" record, don't want to do this until build is complete.
    setIntegerParam(profileBuild_, 0);
    setIntegerParam(profileBuildState_, PROFILE_BUILD_DONE);
    callParamCallbacks();

    return buildOK ? asynSuccess : asynError;
}

asynStatus Automation1MotorController::executeProfile()
{
    bool executeOK = true;

    setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_MOVE_START);
    setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_UNDEFINED);
    setStringParam(profileExecuteMessage_, "");
    callParamCallbacks();
    setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_EXECUTING);

    // This is a bit of a workaround to make the Automation1 controller record the correct data signals.
    // No data will actually be recorded here.
    Automation1_DataCollection_Start(controller_, 
                                     dataCollectionConfig_,
                                     Automation1DataCollectionMode_Snapshot);
    Automation1_DataCollection_Stop(controller_);

    // This compiles and runs the Aeroscript file on the controller. Note that this function returns
    // after the program is started, it does not wait for the program to finish.
    if (!Automation1_Task_ProgramRun(controller_,
                                     PROFILE_MOVE_TASK_INDEX,
                                     "epics_profile_move.ascript"))
    {
        executeOK = false;
        logApiError(profileExecuteMessage_, "Failed to run profile move file");
        goto done;
    }

done:

    // If we didn't fail to start the program, we let the polling thread
    // update profileExecuteState_ when we are done.
    if (!executeOK)
    {
        setIntegerParam(profileExecute_, 0);
        setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_FAILURE);
        setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_DONE);
    }
    callParamCallbacks();

    return executeOK ? asynSuccess : asynError;
}

asynStatus Automation1MotorController::abortProfile()
{
    if (!Automation1_Task_ProgramStop(controller_,
                                      PROFILE_MOVE_TASK_INDEX,
                                      PROFILE_MOVE_ABORT_TIMEOUT))
    {
        logApiError("Automation1 profile move failed to abort");
        return asynError;
    }
    return asynSuccess;
}

asynStatus Automation1MotorController::readbackProfile()
{
    int i;
    size_t j;
    int axis;
    bool readbackOK = true;
    int readbackStatus;
    Automation1DataCollectionStatus dataCollectionStatus;
    int numProfileAxes;
    int allResultsLength;
    int allResultsSize;
    double* allResults = NULL;
    int readPoints = 0;
    int signalResultsSize;
    double* signalResults = NULL;
    int dataPointSpacing;
    double result;
    static const char *functionName="readbackProfile";
    
    setIntegerParam(profileReadbackState_, PROFILE_READBACK_BUSY);
    setIntegerParam(profileReadbackStatus_, PROFILE_STATUS_UNDEFINED);
    setStringParam(profileReadbackMessage_, "");
    callParamCallbacks();

    numProfileAxes = profileAxes_.size();

    // Continuously poll the controller until data collection is done.
    Automation1_DataCollection_GetStatus(controller_, &dataCollectionStatus);
    while (dataCollectionStatus.IsCollecting)
    {
        if (!Automation1_DataCollection_GetStatus(controller_, &dataCollectionStatus))
        {
            readbackOK = false;
            logApiError(profileReadbackMessage_, "Failed to get data collection status");
            goto done;
        }
    }

    readPoints = dataCollectionStatus.NumberOfRetrievedPoints;
    allResultsLength = readPoints * profileAxes_.size() * 2;
    allResultsSize = allResultsLength * sizeof(double);
    allResults = (double*)malloc(allResultsSize);
    
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: readPoints = %i, allResultsLength = %i, allResultsSize = %i\n", driverName, functionName, readPoints, allResultsLength, allResultsSize);    
    
    // Don't cap readPoints at maxProfilePoints_; read all the points that were recorded.
    if (readPoints > numDataPoints_)
    {
       // TODO: Print a warning/error message
       asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: readPoints = %i but expected only %i points\n", driverName, functionName, readPoints, numDataPoints_);    

    }
    // If readPoints isn't a multiple of maxProfilePoints_, dataPointSpacing will be truncated.
    // This may or may not be a problem.
    dataPointSpacing = readPoints / maxProfilePoints_;
    
    signalResultsSize = readPoints * sizeof(double);
    signalResults = (double*)malloc(signalResultsSize);

    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: dataPointSpacing = %i, signalResultsSize = %i\n", driverName, functionName, dataPointSpacing, signalResultsSize);    
    
    // Clear the readback and following error arrays
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
        logApiError(profileReadbackMessage_, "Failed to get data collection results");
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
            logApiError("Failed to parse program position feedback results");
            goto done;
        }
        
        /*
         * There are many more data collection points than both triggers and profile waypoints.
         * For now, provide the maximum amount of data that will fit into existing arrays.
         */
        for (j = 0; j < maxProfilePoints_; j++)
        {
            // Every (numDataPoints_ / maxProfilePoints_)th point should be copied to profileReadbacks_
            result = signalResults[j * dataPointSpacing] * pAxes_[axis]->countsPerUnitParam_;
            pAxes_[axis]->profileReadbacks_[j] = result;
        }

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
            logApiError(profileReadbackMessage_, "Failed to parse position error results");
            goto done;
        }
        
        /*
         * There are many more data collection points than both triggers and profile waypoints.
         * For now, provide the maximum amount of data that will fit into existing arrays.
         */
        for (j = 0; j < maxProfilePoints_; j++)
        {
            // Every (numDataPoints_ / maxProfilePoints_)th point should be copied to profileReadbacks_
            result = signalResults[j * dataPointSpacing] * pAxes_[axis]->countsPerUnitParam_;
            pAxes_[axis]->profileFollowingErrors_[j] = result;
        }
    }

done:

    free(allResults);
    free(signalResults);
    setIntegerParam(profileNumReadbacks_, maxProfilePoints_);

    for (i = 0; i < numProfileAxes; i++)
    {
        axis = profileAxes_[i];
        pAxes_[axis]->readbackProfile();
    }

    readbackStatus = readbackOK ? PROFILE_STATUS_SUCCESS : PROFILE_STATUS_FAILURE;
    setIntegerParam(profileReadbackStatus_, readbackStatus);

    // Clear readback command.  This is a "busy" record, don't want to do this until readback is complete.
    setIntegerParam(profileReadback_, 0);
    setIntegerParam(profileReadbackState_, PROFILE_READBACK_DONE);
    callParamCallbacks();

    return readbackOK ? asynSuccess : asynError;
}

asynStatus Automation1MotorController::poll()
{
    Automation1DataCollectionStatus dataCollectionStatus;
    Automation1TaskStatus taskStatusArray[PROFILE_MOVE_TASK_INDEX + 1];
    Automation1TaskStatus* taskStatus;
    int executeState = PROFILE_EXECUTE_DONE;
    int numPoints;
    int currentPoint;
    bool pollOk = true;

    if (!dataCollectionConfig_) goto done;

    getIntegerParam(profileNumPoints_, &numPoints);
    getIntegerParam(profileExecuteState_, &executeState);
    if (executeState == PROFILE_EXECUTE_EXECUTING)
    {
        if (!Automation1_DataCollection_GetStatus(controller_, &dataCollectionStatus))
        {
            logApiError(profileExecuteMessage_, "Failed to get data collection status");
            pollOk = false;
            goto done;
        }

        if (!Automation1_Task_GetStatus(controller_, taskStatusArray, PROFILE_MOVE_TASK_INDEX + 1)) {
            logApiError("Failed to get task status");
            pollOk = false;
            goto done;
        }
        taskStatus = &taskStatusArray[PROFILE_MOVE_TASK_INDEX];

        currentPoint = dataCollectionStatus.NumberOfRetrievedPoints / displayPointSpacing_;
        if (currentPoint > numPoints)
        {
            currentPoint = numPoints;
        }
        setIntegerParam(profileCurrentPoint_, currentPoint);

        if (taskStatus->TaskState != Automation1TaskState_ProgramRunning) {
            setIntegerParam(profileExecute_, 0);
            setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_DONE);

            if (taskStatus->Error != 0)
            {
                logError(profileExecuteMessage_, "Profile run failed. Task Error: (%d) \"%s\"",
                         taskStatus->Error,
                         taskStatus->ErrorMessage);
                setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_FAILURE);
                goto done;
            }

            setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_SUCCESS);
        }
    }

done:
    if (executeState == PROFILE_EXECUTE_EXECUTING)
    {
        callParamCallbacks();
    }

    return pollOk ? asynSuccess : asynError;
}


/** Logs an driver error and error details from the C API.  Made to reduce duplicate code.
  * \param[in] driverMessage A char array meant to convey where in execution the error occured.
*/
void Automation1MotorController::logApiError(int messageIndex, const char* driverMessage)
{
    int errorCode = Automation1_GetLastError();
    char errorMessage[1024];
    Automation1_GetLastErrorMessage(errorMessage, 1024);

    logError(messageIndex, "%s. API Error: %d \"%s\"",
            driverMessage,
            errorCode,
            errorMessage);
}

void Automation1MotorController::logErrorV(int messageIndex, const char* fmt, std::va_list args)
{

    char buffer[4096];
    vsnprintf(buffer, 4096, fmt, args);

    if (messageIndex != -1) {
        setStringParam(messageIndex, buffer);
    }
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "[Automation1 Driver] %s\n", buffer);
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
