/*************************************************************************\
* Copyright (c) 2021 Aerotech, Inc.
* This file is distributed subject to a Software License Agreement found
* in the file LICENSE that is included with this distribution.
\*************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
// for debugging
#include <fstream>

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
    //
    createParam(AUTOMATION1_PM_PulseModeString,    asynParamInt32,        &AUTOMATION1_PM_PulseMode_);
    createParam(AUTOMATION1_PM_PulsePosString,     asynParamFloat64Array, &AUTOMATION1_PM_PulsePos_);
    createParam(AUTOMATION1_PM_NumPulsesString,    asynParamInt32,        &AUTOMATION1_PM_NumPulses_);
    createParam(AUTOMATION1_PM_PulseDirString,     asynParamInt32,        &AUTOMATION1_PM_PulseDir_);
    createParam(AUTOMATION1_PM_PulseLenString,     asynParamFloat64,      &AUTOMATION1_PM_PulseLen_);
    createParam(AUTOMATION1_PM_PulseSrcString,     asynParamInt32,        &AUTOMATION1_PM_PulseSrc_);
    createParam(AUTOMATION1_PM_PulseAxisString,    asynParamInt32,        &AUTOMATION1_PM_PulseAxis_);
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

/** Called when asyn clients call pasynFloat64Array->write().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Pointer to the array to write.
  * \param[in] nElements Number of elements to write. */
asynStatus Automation1MotorController::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                                  size_t nElements)
{
    int function = pasynUser->reason;
    asynMotorAxis *pAxis;
    asynStatus status = asynSuccess;
    static const char *functionName = "writeFloat64Array";
    
    pAxis = getAxis(pasynUser);
    if (!pAxis) return asynError;
    
    if (function == AUTOMATION1_PM_PulsePos_) {
        // Just copy the positions for now; calculations on the positions will occur when the profile is built
        memcpy(profilePulses_, value, nElements*sizeof(double));
        // Do we want to store nElements?  The user might change (shorten) the number of pulses after writing the array.
        numPulses_ = nElements;
    } 
    else {
        status = asynMotorController::writeFloat64Array(pasynUser, value, nElements);
    }
    return status;
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


asynStatus Automation1MotorController::initializeProfile(size_t maxProfilePoints, size_t maxProfilePulses)
{
    // Initialize max pulses array here, since the base class method only initializes the time, position, readback, and following-error arrays
    maxProfilePulses_ = maxProfilePulses;
    if (profilePulses_) free(profilePulses_);
    profilePulses_ = (double *)calloc(maxProfilePulses, sizeof(double));
    if (profilePulseDisplacements_) free(profilePulseDisplacements_);
    profilePulseDisplacements_ = (double *)calloc(maxProfilePulses, sizeof(double));
    
    return asynMotorController::initializeProfile(maxProfilePoints);
}

asynStatus Automation1MotorController::buildProfile()
{
    int i, j, idx;
    bool buildOK = true;
    int numPoints;
    int numPulses;
    int startPulses;
    int endPulses;
    int timeMode;
    int buildStatus;
    int moveMode;
    int numUsedAxes;
    int useAxis;
    double timePerPoint;
    double totalTime;
    double accelerationTime;
    double segmentTime;
    double distance;
    double preVelocity[MAX_AUTOMATION1_AXES];
    double postVelocity[MAX_AUTOMATION1_AXES];
    // PSO stuff
    int pulseMode;
    //double* pulsePos;
    int pulseDir;
    double pulseLen;
    int pulseSrc;
    int pulseAxis;
    
    std::string profileMoveFileContents;
    Automation1MotorAxis* axis;
    // for troubleshooting
    std::ofstream file("epics_profile_move.ascript");
    static const char *functionName="buildProfile";

    setIntegerParam(profileBuildState_, PROFILE_BUILD_BUSY);
    setIntegerParam(profileBuildStatus_, PROFILE_STATUS_UNDEFINED);
    setStringParam(profileBuildMessage_, "");
    getIntegerParam(profileNumPoints_, &numPoints);
    getIntegerParam(profileNumPulses_, &numPulses);
    getIntegerParam(profileNumPulses_, &startPulses);
    getIntegerParam(profileNumPulses_, &endPulses);
    getIntegerParam(profileTimeMode_, &timeMode);
    getDoubleParam(profileFixedTime_, &timePerPoint);
    // Acceleration is actually acceleration time (unit: seconds)
    getDoubleParam(profileAcceleration_, &accelerationTime);
    getIntegerParam(profileMoveMode_, &moveMode);
    // PSO stuff
    getIntegerParam(AUTOMATION1_PM_PulseMode_, &pulseMode);
    //int AUTOMATION1_PM_PulsePos_;
    getIntegerParam(AUTOMATION1_PM_PulseDir_, &pulseDir);
    getDoubleParam(AUTOMATION1_PM_PulseLen_, &pulseLen);
    getIntegerParam(AUTOMATION1_PM_PulseSrc_, &pulseSrc);
    getIntegerParam(AUTOMATION1_PM_PulseAxis_, &pulseAxis);    
    
    callParamCallbacks();

    // Call the base class method to initialize the time array.  In fixed mode this sets each element to the value of the profileFixedTime_ parameter.
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
        preVelocity[i] = 0.0;
        postVelocity[i] = 0.0;
    }
    profileAxes_.shrink_to_fit();
    numUsedAxes = profileAxes_.size();
    
    /*
     * TODO: where to check for max acceleration? (min acceleration?)
     */
    
    // calculate the pre, post, and total distance
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
    {
        for (i = 0; i < numUsedAxes; i++)
        {
            idx = profileAxes_[i];
            axis = pAxes_[idx];
            
            // In absolute move mode, the difference between the first two profile points is the distance for the first segment
            distance = axis->profilePositions_[1] - axis->profilePositions_[0];
            preVelocity[idx] = distance / profileTimes_[0];
            
            axis->profilePreDistance_ = 0.5 * preVelocity[idx] * accelerationTime;
            axis->profilePrePosition_ = axis->profilePositions_[0] - axis->profilePreDistance_;
            
            // In absolute move mode, the difference between the last two profile points is the distance of the last segment
            distance = axis->profilePositions_[numPoints-1] - 
                       axis->profilePositions_[numPoints-2];
            // The 2nd-to-last profile time is used, since num segments = num points - 1
            postVelocity[idx] = distance / profileTimes_[numPoints-2];

            axis->profilePostDistance_ = 0.5 * postVelocity[idx] * accelerationTime;
            axis->profilePostPosition_ = axis->profilePositions_[numPoints-1] + axis->profilePostDistance_;
            
            /*
             *  The total user-specified distance is needed for the PSO calculation
             */
            // The total distance is the difference between the last and first positions
            axis->profileTotalDistance_ = axis->profilePositions_[numPoints-1] - axis->profilePositions_[0];
        }
    }
    else
    {
        for (i = 0; i < numUsedAxes; i++)
        {
            idx = profileAxes_[i];
            axis = pAxes_[idx];
            
            // In relative move mode, the first position is the distance for the first segment
            distance = axis->profilePositions_[0];
            preVelocity[idx] = distance / profileTimes_[0];
            
            axis->profilePreDistance_ = 0.5 * preVelocity[idx] * accelerationTime;
            axis->profilePrePosition_ = -axis->profilePreDistance_;
            
            // In relative move mode, the second-to-last position is the distance for the last segment and the last point should be ignored
            distance = axis->profilePositions_[numPoints-2];
            // The 2nd-to-last profile time is used, since num segments = num points - 1
            postVelocity[idx] = distance / profileTimes_[numPoints-2];
            
            axis->profilePostDistance_ = 0.5 * postVelocity[idx] * accelerationTime;
            axis->profilePostPosition_ = axis->profilePostDistance_;
            
            /*
             *  The total user-specified distance is needed for the PSO calculation
             */
            // The total distance is the sum of all the displacements
            axis->profileTotalDistance_ = 0.0;
            for (j=0; j < (numPoints-1); j++)
            {
                axis->profileTotalDistance_ += axis->profilePositions_[j];
            }
        }
    }
    
    // The data collection points will be based on the total trajectory time and the 1kHz collection frequency
    // The profile trajectory will usually have many fewer points in it.
    if (timeMode == PROFILE_TIME_MODE_FIXED)
    {
        // The total time should be timePerPoint * numSegments, where numSegments = numPoints - 1
        totalTime = timePerPoint * (numPoints-1);
    }
    else
    {
        totalTime = 0;
        // The total time omits the last point, which is why (numPoints-1) is used
        for (i=0; i < (numPoints-1); i++)
        {
            totalTime += profileTimes_[i];
        }
    }
    // Add the time ramping up/down
    totalTime += (accelerationTime * 2);
    // There is some overhead which results in the data collection stopping before the end of the scan
    //totalTime += 1.0;
    
    // The lowest data collection frequency is currently 1kHz. Arbitrary frequencies will likely be supported in the future.
    numDataPoints_ = totalTime * 1000;
    displayPointSpacing_ = numDataPoints_ / numPoints;
    if (displayPointSpacing_ < 1)
    {
        // Don't let truncation of integer values result in divide-by-zero errors
        displayPointSpacing_ = 1;
    }
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
    
    // Do we have all the info we need to prepare the PSO here?
    // IAMHERE
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: pulseMode = %i, pulseAxis = %i\n", driverName, functionName, pulseMode, pulseAxis);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: numPulses = %i, numPulses_ = %i\n", driverName, functionName, numPulses, numPulses_);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: pulseSrc = %i, pulseDir = %i, pulseLen = %lf\n", driverName, functionName, pulseSrc, pulseDir, pulseLen);   
    
    // Note: numPulses is used instead of numPulses_ (the number of elements written to the pulse position array)
    // because the user might want to reduce the number of pulses without changing the array.
    
    
    /*
     * Turn the profilePulses_ array into an array of displacements in the correct format
     * 
     * Notes: 
     *   * user specified positions will be in EPICS user units
     *   * controller pulse displacements start at the pre profile position, NOT the user-specified start position
     */
    // User-specified positions will be in EPICS user units
    axis = pAxes_[pulseAxis];
    if (pulseMode == 0)
    {
        /* Fixed Mode (divide trajectory by numPulses) */
        
        double pulseSpacing;
        // Assume that numPulses isn't zero for now
        pulseSpacing = axis->profileTotalDistance_ / numPulses;
        
        // The 1st pulse always occurs after the pre profile displacement 
        profilePulseDisplacements_[0] = axis->profilePreDistance_;
        // The remaining pulses are evenly spaced
        for (i=1; i < numPulses; i++)
        {
            profilePulseDisplacements_[i] = pulseSpacing;
        }
    } else if (pulseMode == 1)
    {
        /* Array Mode (convert array into displacements) */

        if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
        {
            profilePulseDisplacements_[0] = profilePulses_[0] - axis->profilePrePosition_;
            for (i=1; i < numPulses; i++)
            {
                profilePulseDisplacements_[i] = profilePulses_[i] - profilePulses_[i-1];
            }
        }
        else
        {
            profilePulseDisplacements_[0] = axis->profilePreDistance_ + profilePulses_[0];
            for (i=1; i < numPulses; i++)
            {
                profilePulseDisplacements_[i] = profilePulses_[i];
            }
        }
    } else if (pulseMode == 2)
    {
        /* TrajPts Mode (convert traj point positions into displacements) */
        
        if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
        {
            double totalPulseDistance, pulseSpacing;
            totalPulseDistance = axis->profilePositions_[endPulses] - axis->profilePositions_[startPulses];
            pulseSpacing = totalPulseDistance / numPulses;
            
            profilePulseDisplacements_[0] = axis->profilePositions_[startPulses] - axis->profilePrePosition_;
            for (i=1; i < numPulses; i++)
            {
                profilePulseDisplacements_[i] = profilePulseDisplacements_[i-1] + pulseSpacing;
            }
        }
        else
        {
            profilePulseDisplacements_[0] = axis->profilePreDistance_;
            // Does startPulses = n mean to send a pulse at the start or end of the nth segment?  Assume end for now.
            for (i=0; i <= startPulses; i++)
            {
                profilePulseDisplacements_[0] += axis->profilePositions_[i];
            }
            
            double remainingPulseDistance, pulseSpacing;
            remainingPulseDistance = 0.0;
            for (i=startPulses+1; i <= endPulses; i++)
            {
                remainingPulseDistance += axis->profilePositions_[i];
            }
            
            pulseSpacing = remainingPulseDistance / numPulses;
            
            for (i=0; i < numPulses; i++)
            {
                profilePulseDisplacements_[i] = pulseSpacing;
            }
        }
    } else if (pulseMode == 3)
    {
        // None Mode - what should be done in this case?  Set numPulses to zero?  Zero the pulse array?
    } 
    
    
    /*
     * Commands sent by EnsembleTrajectoryScan.st
     * 
    PSOCONTROL @0 RESET
    #
    PSOTRACK @0 INPUT 3
    #
    PSOARRAY @0,50,8000
    #
    PSOPULSE @0 TIME 1.50,1.00
    #
    PSOOUTPUT @0 PULSE
    #
    PSOTRACK @0 DIRECTION 2
    #
    PSODISTANCE @0 ARRAY
    #
    PSOCONTROL @0 ARM
    #
    */ 
    
    
    
    
    
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
    
    // TODO: Configure PSO here
        
    // We start data collection just before the actual profile moves.
    profileMoveFileContents.append("AppDataCollectionSnapshot()\n");

    // This block assembled the main part of the program.  Each profile time corresponds to one
    // movePtCommand in Aereoscript.
    // i = -1 => ramp up ; i = numPoints => ramp down
    for (i = -1; i <= numPoints; i++)
    {
        if (i == numPoints-1)
        {
            // Skip the last user-specified point; it isn't meaningful in relative mode since there are only n-1 segments.
            // In absolute mode the (numPoints-1)th position was already included in the (numPoints-2)th point.
            continue;
        }
        profileMoveFileContents.append("MovePt($axes, [");
        for (j = 0; j < numUsedAxes; j++)
        {
            axis = pAxes_[profileAxes_[j]];
            if (i == -1)
            {
                // Ramp up before starting user-specified profile
                if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
                {
                    // The ramp up ends at the first (0th) user-specified point
                    profileMoveFileContents.append(std::to_string(axis->profilePositions_[i+1] / axis->countsPerUnitParam_));
                }
                else
                {
                    // The ramp up ends at after returning the pre distance
                    profileMoveFileContents.append(std::to_string(axis->profilePreDistance_ / axis->countsPerUnitParam_));
                }
                // The ramp up period always uses the user-specified acceleration time
                segmentTime = accelerationTime;
                
            }
            else if (i == numPoints)
            {
                // Ramp down after completing user-specified profile
                if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
                {
                    // The ramp down ends at the post position
                    profileMoveFileContents.append(std::to_string(axis->profilePostPosition_ / axis->countsPerUnitParam_));
                }
                else
                {
                    // The ramp down ends at after traveling the post distance
                    profileMoveFileContents.append(std::to_string(axis->profilePostDistance_ / axis->countsPerUnitParam_));
                }
                // The ramp down period always uses the user-specified acceleration time
                segmentTime = accelerationTime;
            }
            else
            {
                if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
                {
                    // The position for the (i)th segement is the end point for that segment, which is the (i+1)th position
                    profileMoveFileContents.append(std::to_string(axis->profilePositions_[i+1] / axis->countsPerUnitParam_));
                }
                else
                {
                    // The displacement for th (i)th segment is the (i)th position
                    profileMoveFileContents.append(std::to_string(axis->profilePositions_[i] / axis->countsPerUnitParam_));
                }
                segmentTime = profileTimes_[i];
            }
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
        profileMoveFileContents.append(std::to_string(segmentTime * 1000));
        profileMoveFileContents.append(")\n");
    }

    profileMoveFileContents.append("WaitForMotionDone($axes)\n");
    profileMoveFileContents.append("AppDataCollectionStop()\n");
    profileMoveFileContents.append("ParameterSetTaskValue(");
    profileMoveFileContents.append(std::to_string(PROFILE_MOVE_TASK_INDEX));
    profileMoveFileContents.append(", TaskParameter.MotionInterpolationMode, $motionInterpolationMode)\n");
    profileMoveFileContents.append("end");
    
    // Write the file to the IOC's startup dir for troubleshooting
    //file << profileMoveFileContents;
    //file.close();
    
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
    int i;
    int moveMode;
    Automation1MotorAxis* axis;
    double motorRecVelocity;
    double defaultVelocity;
    //double defaultCoordVelocity;
    int* axes;
    int numUsedAxes;
    double *positions;
    double *velocities;
    static const char *functionName="executeProfile";

    getIntegerParam(profileMoveMode_, &moveMode);
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
    
    /*
     * Move motors to the start position
     */
    
    axes = &profileAxes_[0];
    numUsedAxes = profileAxes_.size();
    positions = (double*)calloc(numUsedAxes, sizeof(double));
    velocities = (double*)calloc(numUsedAxes, sizeof(double));
    
    // Determine starting positions and velocities
    for (i = 0; i < numUsedAxes; i++)
    {
        // Collect the starting positions
        axis = pAxes_[profileAxes_[i]];
        
        // The move mode was already taken into account when calculating the profilePrePosition_
        positions[i] = axis->profilePrePosition_ / axis->countsPerUnitParam_;
        
        // Query motor record and default axis velocities
        getDoubleParam(motorVelocity_, profileAxes_[i], &motorRecVelocity);
        Automation1_Parameter_GetAxisValue(controller_, profileAxes_[i], Automation1AxisParameterId_DefaultAxisSpeed, &defaultVelocity);
        // The default coordinated speed exceeded the max speed for the axis used during development
        //Automation1_Parameter_GetTaskValue(controller_, profileAxes_[i], Automation1TaskParameterId_DefaultCoordinatedSpeed, &defaultCoordVelocity);
        
        // Set the velocity to a reasonable value
        if (motorRecVelocity != 0.0)
        {
            // if a move was made using the motor record since the IOC started, motorRecVelocity should be non-zero, so we'll use it.
            velocities[i] = motorRecVelocity;
        }
        else
        {
            // if a move hasn't been commanded using the motor record since the IOC started, use the default axis speed in the controller instead.
            velocities[i] = defaultVelocity;
        }
        asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: axis = %i, motorRecVelocity = %lf, defaultVelocity = %lf\n", driverName, functionName, profileAxes_[i], motorRecVelocity, defaultVelocity);
    }
    
    if (moveMode == PROFILE_MOVE_MODE_ABSOLUTE)
    {
        // Automation1_Command_MoveAbsolute(Automation1Controller controller, int32_t executionTaskIndex, int32_t* axes, int32_t axesLength, double* positions, int32_t positionsLength, double* speeds, int32_t speedsLength);
        Automation1_Command_MoveAbsolute(controller_, 1, axes, numUsedAxes, positions, numUsedAxes, velocities, numUsedAxes);
    }
    else
    {
        // Automation1_Command_MoveIncremental(Automation1Controller controller, int32_t executionTaskIndex, int32_t* axes, int32_t axesLength, double* distances, int32_t distancesLength, double* speeds, int32_t speedsLength);
        Automation1_Command_MoveIncremental(controller_, 1, axes, numUsedAxes, positions, numUsedAxes, velocities, numUsedAxes);
    }
    // Automation1_Command_WaitForMotionDone(Automation1Controller controller, int32_t executionTaskIndex, int32_t* axes, int32_t axesLength);
    Automation1_Command_WaitForMotionDone(controller_, 1, axes, numUsedAxes);
    
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
    
    free(positions);
    free(velocities);
    
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
    double dataPointSpacing;
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
    // dataPointSpacing is a double and is unlikely to be an integer
    dataPointSpacing = 1.0 * readPoints / maxProfilePoints_;
    
    signalResultsSize = readPoints * sizeof(double);
    signalResults = (double*)malloc(signalResultsSize);

    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s:%s: dataPointSpacing = %lf, signalResultsSize = %i\n", driverName, functionName, dataPointSpacing, signalResultsSize);    
    
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
            result = signalResults[(int)floor((j * dataPointSpacing)+0.5)] * pAxes_[axis]->countsPerUnitParam_;
            pAxes_[axis]->profileReadbacks_[j] = result;
            // Uncomment for troubleshooting
            //asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: j = %i ; pos = %lf\n", driverName, functionName, (int) floor((j*dataPointSpacing)+0.5), signalResults[(int)floor((j * dataPointSpacing)+0.5)]);    

        }
        // Uncomment for troubleshooting
        //asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: last recorded data point = %lf, index = %i\n", driverName, functionName, signalResults[readPoints-1], readPoints-1);    
        
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
            result = signalResults[(int)floor((j * dataPointSpacing)+0.5)] * pAxes_[axis]->countsPerUnitParam_;
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

asynStatus Automation1CreateProfile(const char *portName, int maxPoints, int maxPulses)
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
    pC->initializeProfile(maxPoints, maxPulses);
    return asynSuccess;
}

// Profile Setup arguments
static const iocshArg Automation1CreateProfileArg0 = {"Port name", iocshArgString};
static const iocshArg Automation1CreateProfileArg1 = {"Max points", iocshArgInt};
static const iocshArg Automation1CreateProfileArg2 = {"Max pulses", iocshArgInt};

static const iocshArg* const Automation1CreateProfileArgs[3] = {&Automation1CreateProfileArg0, &Automation1CreateProfileArg1, &Automation1CreateProfileArg2};

static const iocshFuncDef configAutomation1Profile = {"Automation1CreateProfile", 3, Automation1CreateProfileArgs};

static void configAutomation1ProfileCallFunc(const iocshArgBuf* args)
{
    Automation1CreateProfile(args[0].sval, args[1].ival, args[2].ival);
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
