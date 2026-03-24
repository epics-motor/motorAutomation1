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
#include "Automation1MotorAxis.h"
#include "Automation1MotorController.h"
#include "Include/Automation1.h"

static const char *driverName = "Automation1MotorAxis";

/** Creates a new Automation1 axis object.
  * \param[in] pC     Pointer to the Automation1MotorController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  */
Automation1MotorAxis::Automation1MotorAxis(Automation1MotorController* pC, int axisNo)
    : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
    fullProfilePositions_ = NULL;
    Automation1ConfiguredParameters configuredParameters;
    double direction;
    
    Automation1_StatusConfig_Create(&(statusConfig_));
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_AxisStatus, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_DriveStatus, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_ProgramPositionFeedback, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_ProgramVelocityFeedback, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_AxisFault, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_PositionError, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_ProgramPosition, 0);

    // Determine if this axis has reverse direction of encoder and motor direction
    if (!Automation1_ConfiguredParameters_Create(&configuredParameters)) {
      logError("Error calling ConfiguredParameters_Create");
      return;
    }
  
    if (!Automation1_Configuration_GetConfiguredParameters(pC_->controller_, configuredParameters)) {  
      logError("Error calling GetConfiguredParameters");
      return;
    }
    
    if (!Automation1_ConfiguredParameters_GetAxisValue(configuredParameters, axisNo_, 
         Automation1AxisParameterId_ReverseMotionDirection, &direction)) {
      logError("Error calling ConfiguredParameters_GetAxisValue");
      return;
    }
    reverseDirection_ = (direction != 0);

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC->motorStatusGainSupport_, 1);
}

// Destructor.
Automation1MotorAxis::~Automation1MotorAxis()
{
    Automation1_StatusConfig_Destroy(statusConfig_);
}

/** Reports on status of the axis
  * \param[in] fp    The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void Automation1MotorAxis::report(FILE* fp, int level)
{
    if (level > 0) {
        fprintf(fp, "    axis %d\n", axisNo_);
    }

    // Call the base class method
    asynMotorAxis::report(fp, level);
}

/** Move the motor by a relative amount or to an absolute position.
  * \param[in] position     The absolute position to move to (if relative=0) or the relative distance to move
  *                          by (if relative=1). Units=steps.
  * \param[in] relative     Flag indicating relative move (1) or absolute move (0).
  * \param[in] minVelocity  The initial velocity. Not used by Automation1.
  * \param[in] maxVelocity  The target velocity for the move.  Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  */
asynStatus Automation1MotorAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
    double adjustedAcceleration = acceleration / countsPerUnitParam_;
    double adjustedVelocity = maxVelocity / countsPerUnitParam_;
    double adjustedPosition = position / countsPerUnitParam_;
    bool moveSuccessful;
    
    setIntegerParam(pC_->motorStatusProblem_,0);            // Unset "Problem" status bit. A logError call will set it.

    if (!Automation1_Command_SetupAxisRampValue(pC_->controller_,
                                                1,
                                                axisNo_,
                                                Automation1RampMode_Rate,
                                                adjustedAcceleration,
                                                Automation1RampMode_Rate,
                                                adjustedAcceleration))
    {
        logError("Failed to set acceleration prior to move.");
        return asynError;
    }

    // Note that MoveIncremental and MoveAbsolute are non-blocking functions.
    if (relative)
    {
        moveSuccessful = Automation1_Command_MoveIncremental(pC_->controller_,
                                                            1,
                                                            &axisNo_,
                                                            1,
                                                            &adjustedPosition,
                                                            1,
                                                            &adjustedVelocity,
                                                            1);
    }
    else
    {
        moveSuccessful = Automation1_Command_MoveAbsolute(pC_->controller_,
                                                          1,
                                                          &axisNo_,
                                                          1,
                                                          &adjustedPosition,
                                                          1,
                                                          &adjustedVelocity,
                                                          1);
    }

    if (moveSuccessful)
    {
        return asynSuccess;
    }
    else
    {
        logError("Failed to move axis.");
        return asynError;
    }
}

/** Move the motor to the home position.
  * \param[in] minVelocity  The initial velocity. Not used by Automation1.
  * \param[in] maxVelocity  The target velocity for the home. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  * \param[in] forwards     Flag indicating motor direction. Not used by Automation1.
  */
asynStatus Automation1MotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
    double adjustedAcceleration = acceleration / countsPerUnitParam_;
    
    setIntegerParam(pC_->motorStatusProblem_,0);            // Unset "Problem" status bit. A logError call will set it.

    if (!Automation1_Command_SetupAxisRampValue(pC_->controller_,
                                                1,
                                                axisNo_,
                                                Automation1RampMode_Rate,
                                                adjustedAcceleration,
                                                Automation1RampMode_Rate,
                                                adjustedAcceleration))
    {
        logError("Failed to set acceleration prior to home.");
        return asynError;
    }

    if (Automation1_Command_HomeAsync(pC_->controller_, 1, &axisNo_, 1))
    {
        return asynSuccess;
    }
    else
    {
        logError("Failed to home axis.");
        return asynError;
    }
}

/** Move the motor at a fixed velocity until told to stop.
  * \param[in] minVelocity  The initial velocity. Not used by Automation1.
  * \param[in] maxVelocity  The target velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  */
asynStatus Automation1MotorAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
    double adjustedAcceleration = acceleration / countsPerUnitParam_;
    double adjustedVelocity = maxVelocity / countsPerUnitParam_;
    
    setIntegerParam(pC_->motorStatusProblem_,0);            // Unset "Problem" status bit. A logError call will set it.

    if (!Automation1_Command_SetupAxisRampValue(pC_->controller_,
                                                1,
                                                axisNo_,
                                                Automation1RampMode_Rate,
                                                adjustedAcceleration,
                                                Automation1RampMode_Rate,
                                                adjustedAcceleration))
    {
        logError("Failed to set acceleration prior to moveVelocity (jog).");
        return asynError;
    }

    if (Automation1_Command_MoveFreerun(pC_->controller_, 1, &axisNo_, 1, &adjustedVelocity, 1))
    {
        return asynSuccess;
    }
    else
    {
        logError("moveVelocity (jog) failed.");
        return asynError;
    }
}

/** Stop the motor.
  * \param[in] acceleration The acceleration value. In the case of aborting motion in Automation1,
                            acceleration is determined by a separate controller parameter, so 
                            this is not used.
  */
asynStatus Automation1MotorAxis::stop(double acceleration)
{
    setIntegerParam(pC_->motorStatusProblem_,0);            // Unset "Problem" status bit. A logError call will set it.
    
    if (Automation1_Command_Abort(pC_->controller_, &axisNo_, 1))
    {
        return asynSuccess;
    }
    else
    {
        logError("Failed to stop axis.");
        return asynError;
    }
}

/** Set the current position of the motor.
  * \param[in] position The new absolute motor position that should be set in the hardware. Units=steps.
  */
asynStatus Automation1MotorAxis::setPosition(double position)
{
    double adjustedPosition = position / countsPerUnitParam_;
    setIntegerParam(pC_->motorStatusProblem_,0);            // Unset "Problem" status bit. A logError call will set it.

    if (Automation1_Command_PositionOffsetSet(pC_->controller_, 1, &axisNo_, 1, &adjustedPosition, 1))
    {
        return asynSuccess;
    }
    else
    {
        logError("Failed to set position of axis.");
        return asynError;
    }
}

/** Enables or disables the motor.
  * \param[in] closedLoop true = enable, false = disable.
  */
asynStatus Automation1MotorAxis::setClosedLoop(bool closedLoop)
{
    if (closedLoop)
    {
        // Automatically attempt to clear faults before attempting to enable the motor
        if(!Automation1_Command_AcknowledgeAll(pC_->controller_, pC_->commandExecuteTask_))
        {
            logError("Could not clear faults.");
        }
        
        if (!Automation1_Command_Enable(pC_->controller_, pC_->commandExecuteTask_, &axisNo_, 1))
        {
            logError("Failed to enable axis.");
            return asynError;
        }
    }
    else
    {
        if (!Automation1_Command_Disable(pC_->controller_, &axisNo_, 1))
        {
            logError("Failed to disable axis.");
            return asynError;
        }
    }
    setIntegerParam(pC_->motorStatusProblem_,0);	//Clear problem bit if it was set due to "axis not enabled"
    return asynSuccess;
}


/** Function to define the motor positions for a profile move. 
  * Called when the profileMove positions waveform is processed.
  * \param[in] positions Array of profile positions for this axis in user units.
  * \param[in] numPoints The number of positions in the array.
  */
asynStatus Automation1MotorAxis::defineProfile(double *positions, size_t numPoints)
{
  size_t i;
  static const char *functionName = "defineProfile";
  
  // Call the base class method to convert from EPICS user coordinates to steps
  asynMotorAxis::defineProfile(positions, numPoints);
  
  // Convert from steps to Automation1 units
  for (i=0; i<numPoints; i++)
  {
    profilePositions_[i] = profilePositions_[i] / countsPerUnitParam_;
  }
  
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "%s:%s: axis=%d, countsPerUnitParam_=%f, profilePositions_[0]=%f\n",
            driverName, functionName, axisNo_, countsPerUnitParam_, profilePositions_[0]);
  
  return asynSuccess;
}


/** Polls the axis.
  *
  * This function reads the motor position, encoder position, and the following statuses: direction,
  * done, high and low limit, at home, following error, moving, and comms error.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false).
*/
asynStatus Automation1MotorAxis::poll(bool* moving)
{
    bool pollSuccessfull = true;
    double results[7];
    int axisStatus;
    int driveStatus;
    int enabled;
    double programPositionFeedback;
    double programVelocityFeedback;
    double positionError;
    int axisFaults;
    int done;
    
    // This actually retrieves the status items from the controller.
    if (!Automation1_Status_GetResults(pC_->controller_,
                                       statusConfig_,
                                       results,
                                       (int)(sizeof(results) / sizeof(double))))
    {
        pollSuccessfull = false;
        goto skip;
    }
    axisStatus = results[0];
    driveStatus = results[1];
    programPositionFeedback = results[2];
    programVelocityFeedback = results[3];
    axisFaults = (int)results[4];
    positionError = results[5];
    programPosition_ = results[6];

    asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "Automation1_Status_GetResults(%d): axis status = %d; drive status = %d; position feedback = %lf; velocity feedback %lf\n",
              axisNo_,
              axisStatus,
              driveStatus,
              programPositionFeedback,
              programVelocityFeedback);

    if (!Automation1_Parameter_GetAxisValue(pC_->controller_,
        axisNo_,
        Automation1AxisParameterId_CountsPerUnit,
        &countsPerUnitParam_))
    {
        pollSuccessfull = false;
        goto skip;
    }

    asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "Automation1_Status_GetAxisValue(%d): counts per unit = %lf\n",
              axisNo_,
              countsPerUnitParam_);

    setIntegerParam(pC_->AUTOMATION1_C_AxisStatusBits_, axisStatus);
    setIntegerParam(pC_->AUTOMATION1_C_DriveStatusBits_, driveStatus);

    enabled = driveStatus & Automation1DriveStatus_Enabled;
    setIntegerParam(pC_->motorStatusPowerOn_, enabled);
    setDoubleParam(pC_->motorPosition_, programPositionFeedback * countsPerUnitParam_);
    setDoubleParam(pC_->motorEncoderPosition_, programPositionFeedback * countsPerUnitParam_);

    setDoubleParam(pC_->AUTOMATION1_C_Velocity_, programVelocityFeedback);  //ajc-osl
    setDoubleParam(pC_->AUTOMATION1_C_FError_, positionError * countsPerUnitParam_);

    done = axisStatus & Automation1AxisStatus_MotionDone;
    setIntegerParam(pC_->motorStatusDone_, done);
    if (done)
    {
        *moving = false;
        setIntegerParam(pC_->motorStatusMoving_, 0);
    }
    else
    {
        if (programVelocityFeedback != 0)
        {
            *moving = true;
            setIntegerParam(pC_->motorStatusMoving_, 1);
            if (programVelocityFeedback > 0)
            {
                setIntegerParam(pC_->motorStatusDirection_, 1);
            }
            else
            {
                setIntegerParam(pC_->motorStatusDirection_, 0);
            }
        }
        else
        {
            *moving = false;
            setIntegerParam(pC_->motorStatusMoving_, 0);
        }
    }

    if( axisFaults > 0) setIntegerParam(pC_->motorStatusProblem_,1);            // Set "Problem" status bit if there are any axis faults.

    if ((axisFaults & Automation1AxisFault_CwEndOfTravelLimitFault) ||
        (axisFaults & Automation1AxisFault_CwSoftwareLimitFault))
    {
        setIntegerParam(pC_->motorStatusHighLimit_, 1);
    }
    else
    {
        setIntegerParam(pC_->motorStatusHighLimit_, 0);
    }

    if ((axisFaults & Automation1AxisFault_CcwEndOfTravelLimitFault) ||
        (axisFaults & Automation1AxisFault_CcwSoftwareLimitFault))
    {
        setIntegerParam(pC_->motorStatusLowLimit_, 1);
    }
    else
    {
        setIntegerParam(pC_->motorStatusLowLimit_, 0);
    }

    if (axisFaults & Automation1AxisFault_PositionErrorFault)
    {
        setIntegerParam(pC_->motorStatusFollowingError_, 1);
    }
    else
    {
        setIntegerParam(pC_->motorStatusFollowingError_, 0);
    }

    setIntegerParam(pC_->motorStatusHomed_, axisStatus & Automation1AxisStatus_Homed);

skip:

    setIntegerParam(pC_->motorStatusCommsError_, !pollSuccessfull);
    callParamCallbacks();
    if (!pollSuccessfull)
    {
        logError("Poll failed.");
    }

    return pollSuccessfull ? asynSuccess : asynError;
}

/** Enables or disables position compare output
  *
  * \param[in] enable A flag to enable (true) or disable (false) PCO
*/
asynStatus Automation1MotorAxis::enablePCO(bool enable)
{
  Automation1PsoDistanceInput distanceInput;
  Automation1PsoWindowInput windowInput;
  Automation1PsoOutputPin outputPin;
  int recDirection, dir;
  double recOffset;
  int iTemp;
  int windowNumber = 0;
  int taskNumber = 1;
  double startPosition, endPosition, increment, pulseWidth;

  pC_->getIntegerParam(axisNo_, pC_->AUTOMATION1_PSO_DistanceInput_, &iTemp);
  distanceInput = (Automation1PsoDistanceInput)iTemp;
  pC_->getIntegerParam(axisNo_, pC_->AUTOMATION1_PSO_WindowInput_,   &iTemp);
  windowInput = (Automation1PsoWindowInput)iTemp;
  pC_->getIntegerParam(axisNo_, pC_->AUTOMATION1_PSO_OutputPin_,     &iTemp);
  outputPin = (Automation1PsoOutputPin)iTemp;
  pC_->getDoubleParam(axisNo_,  pC_->PCOStartPosition_, &startPosition);
  pC_->getDoubleParam(axisNo_,  pC_->PCOEndPosition_,   &endPosition);
  pC_->getDoubleParam(axisNo_,  pC_->PCOIncrement_,     &increment);
  pC_->getDoubleParam(axisNo_,  pC_->PCOPulseWidth_,    &pulseWidth);
  pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_, &recDirection);
  pC_->getDoubleParam(axisNo_,  pC_->motorRecOffset_,    &recOffset);

  dir = recDirection ? -1 : 1;
  startPosition = (startPosition - recOffset) * dir;
  endPosition = (endPosition - recOffset) * dir;

  printf("Automation1MotorAxis::enablePCO entry, startPosition=%f, endPosition=%f, increment=%f, pulseWidth=%f," 
         "reverseDirection=%d, distanceInput=%d, windowInput=%d, outputPin=%d, recDirection=%d, recOffset=%f, enable=%d\n",
         startPosition, endPosition, increment, pulseWidth, reverseDirection_, distanceInput, windowInput, outputPin, 
         recDirection, recOffset, enable);

  if (!Automation1_Command_PsoReset(pC_->controller_, taskNumber, axisNo_)) {
    logError("Error calling PsoReset");
    return asynError;
  }
  
  // If enable is 0 we are done
  if (enable == 0) {
    return asynSuccess;
  }

  if (!Automation1_Command_PsoDistanceConfigureInputs(pC_->controller_, taskNumber, axisNo_, &distanceInput, 1)) {
    logError("Error calling PsoDistanceConfigureInputs");
    return asynError;
  }

  if (!Automation1_Command_PsoDistanceConfigureFixedDistance(pC_->controller_, taskNumber, axisNo_, std::lround(increment * countsPerUnitParam_))) {
    logError("Error calling PsoDistanceConfigureFixedDistance");
    return asynError;
  }

  if (!Automation1_Command_PsoDistanceCounterOn(pC_->controller_, taskNumber, axisNo_)) {
    logError("Error calling PsoDistanceCounterOn");
    return asynError;
  }

  if (!Automation1_Command_PsoDistanceEventsOn(pC_->controller_, taskNumber, axisNo_)) {
    logError("Error calling PsoDistanceEventsOn");
    return asynError;
  }

  if (!Automation1_Command_PsoWindowConfigureInput(pC_->controller_, taskNumber, axisNo_, windowNumber, windowInput, reverseDirection_)) {
    logError("Error calling PsoWindowConfigureInput");
    return asynError;
  }

  if (!Automation1_Command_PsoWindowCounterSetValue(pC_->controller_, taskNumber, axisNo_, windowNumber, 
       std::lround(programPosition_ * countsPerUnitParam_))) {
    logError("Error calling PsoWindowCCounterSetValue");
    return asynError;
  }

  double lowerBound = std::lround(std::min(startPosition, endPosition) * countsPerUnitParam_);
  double upperBound = std::lround(std::max(startPosition, endPosition) * countsPerUnitParam_);
  if (!Automation1_Command_PsoWindowConfigureFixedRange(pC_->controller_, taskNumber, axisNo_, windowNumber, lowerBound, upperBound)) {
    logError("Error calling PsoWindowConfigureFixedRange");
    return asynError;
  }

  if (!Automation1_Command_PsoWindowOutputOn(pC_->controller_, taskNumber, axisNo_, windowNumber)) {
    logError("Error calling PsoWindowOutputOn");
    return asynError;
  }

  if (!Automation1_Command_PsoEventConfigureMask(pC_->controller_, taskNumber, axisNo_, Automation1PsoEventMask_WindowMask)) {
    logError("Error calling PsoEventConfigureMask");
    return asynError;
  }

  if (!Automation1_Command_PsoWaveformConfigureMode(pC_->controller_, taskNumber, axisNo_, Automation1PsoWaveformMode_Pulse)) {
    logError("Error calling PsoWaveformConfigureMode");
    return asynError;
  }

  if (!Automation1_Command_PsoWaveformConfigurePulseFixedTotalTime(pC_->controller_, taskNumber, axisNo_, pulseWidth * 1.e6)) {
    logError("Error calling PsoWaveformConfigurePulseFixedTotalTime");
    return asynError;
  }

  if (!Automation1_Command_PsoWaveformConfigurePulseFixedOnTime(pC_->controller_, taskNumber, axisNo_, pulseWidth * 1.e6)) {
    logError("Error calling PsoWaveformConfigurePulseFixedOnTime");
    return asynError;
  }

  if (!Automation1_Command_PsoWaveformConfigurePulseFixedCount(pC_->controller_, taskNumber, axisNo_, 1)) {
    logError("Error calling PsoWaveformConfigurePulseFixedCount");
    return asynError;
  }

  if (!Automation1_Command_PsoWaveformApplyPulseConfiguration(pC_->controller_, taskNumber, axisNo_)) {
    logError("Error calling PsoWaveformApplyPulseConfiguration");
    return asynError;
  }

  if (!Automation1_Command_PsoWaveformOn(pC_->controller_, taskNumber, axisNo_)) {
    logError("Error calling PsoWaveformOn");
    return asynError;
  }

  if (!Automation1_Command_PsoOutputConfigureSource(pC_->controller_, taskNumber, axisNo_, Automation1PsoOutputSource_Waveform)) {
    logError("Error calling PsoOutputConfigureSource");
    return asynError;
  }

  if (!Automation1_Command_PsoOutputConfigureOutput(pC_->controller_, taskNumber, axisNo_, outputPin)) {
    logError("Error calling PsoOutputConfigureOutput");
    return asynError;
  }
  printf("Automation1MotorAxis::enablePCO exit success\n");
  return asynSuccess;

}

/** Logs an driver error and error details from the C API.  Made to reduce duplicate code.
  * \param[in] driverMessage A char array meant to convey where in execution the error occured.
*/
void Automation1MotorAxis::logError(const char* driverMessage)
{
    int errorCode = Automation1_GetLastError();
    char errorMessage[1024];
    Automation1_GetLastErrorMessage(errorMessage, 1024);
    
    setIntegerParam(pC_->motorStatusProblem_,1);            // Set "Problem" status bit for any failed API call including when a disabled axis is commanded.

    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "Driver: Automation1. Function Message: %s. Axis: %d. API Error Code: %d. API Error Message: %s\n",
              driverMessage,
              axisNo_,
              errorCode,
              errorMessage);
    return;
}
