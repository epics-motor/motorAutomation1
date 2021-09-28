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

/** Creates a new Automation1 axis object.
  * \param[in] pC     Pointer to the Automation1MotorController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  */
Automation1MotorAxis::Automation1MotorAxis(Automation1MotorController* pC, int axisNo)
    : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
    Automation1_StatusConfig_Create(&(statusConfig_));
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_AxisStatus, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_ProgramPositionFeedback, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_ProgramVelocityFeedback, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_ProgramPositionCommand, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_AxisFault, 0);

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
    double resolution;
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);
    double adjustedAcceleration = acceleration * resolution;
    double adjustedVelocity = maxVelocity * resolution;
    double adjustedPosition = position * resolution;
    bool moveSuccessful;

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
    double resolution;
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);
    double adjustedAcceleration = acceleration * resolution;

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

    if (Automation1_Command_Home(pC_->controller_, 1, &axisNo_, 1))
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
    double resolution;
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);
    double adjustedAcceleration = acceleration * resolution;
    double adjustedVelocity = maxVelocity * resolution;

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
    double resolution;
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);
    double adjustedPosition = position * resolution;

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
        if (!Automation1_Command_Enable(pC_->controller_, 1, &axisNo_, 1))
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
    double results[5];
    int axisStatus;
    double programPositionFeedback;
    double programVelocityFeedback;
    double programPositionCommand;
    double countsPerUnitParam;
    int axisFaults;
    int done;
    double resolution;
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);

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
    programPositionFeedback = results[1];
    programVelocityFeedback = results[2];
    programPositionCommand = results[3];
    axisFaults = (int)results[4];

    if (!Automation1_Parameter_GetAxisValue(pC_->controller_,
        axisNo_,
        Automation1AxisParameterId_CountsPerUnit,
        &countsPerUnitParam))
    {
        pollSuccessfull = false;
        goto skip;
    }

    setDoubleParam(pC_->motorPosition_, programPositionFeedback / resolution);
    setDoubleParam(pC_->motorEncoderPosition_, programPositionFeedback * countsPerUnitParam);

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

    setIntegerParam(pC_->motorStatusAtHome_, axisStatus & Automation1AxisStatus_Homed);

skip:

    setIntegerParam(pC_->motorStatusCommsError_, pollSuccessfull ? 0 : 1);
    callParamCallbacks();
    if (!pollSuccessfull)
    {
        logError("Poll failed.");
    }

    return pollSuccessfull ? asynSuccess : asynError;
}

/** Function to define the motor positions for a profile move.
  * This function calls the base class method, then converts the positions to
  * controller units.
  * \param[in] positions Array of profile positions for this axis in user units.
  * \param[in] numPoints The number of positions in the array.
  */
asynStatus Automation1MotorAxis::defineProfile(double* positions, size_t numPoints)
{
    double resolution;
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);
    // Call the base class function (converts from EGU to steps)
    asynStatus status = asynMotorAxis::defineProfile(positions, numPoints);
    if (status) return status;

    // Convert from steps to EGU.
    for (size_t i = 0; i < numPoints; i++)
    {
        profilePositions_[i] = profilePositions_[i] * resolution;
    }
    return asynSuccess;
}

asynStatus Automation1MotorAxis::readbackProfile()
{
    // Call the base class method
    asynMotorAxis::readbackProfile();
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

    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "Driver: Automation1. Function Message: %s. Axis: %d. API Error Code: %d. API Error Message: %s\n",
              driverMessage,
              axisNo_,
              errorCode,
              errorMessage);
    return;
}
