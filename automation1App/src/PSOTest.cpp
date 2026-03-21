#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <cmath>
#include "Include/Automation1.h"

#define ENCODER_COUNTS_PER_DEGREE 253770
#define DEGREES_PER_PULSE 0.01
#define AXIS_NUMBER 0
#define TASK_INDEX 1
#define AXIS 0
#define PULSE_WIDTH_SEC .001
#define CLOCK_FREQUENCY 1e6
#define DUTY_CYCLE 0.5
#define WINDOW_NUMBER 0
#define PULSE_SRC Automation1PsoDistanceInput_iXL5ePrimaryFeedback
#define WINDOW_SRC Automation1PsoWindowInput_iXL5ePrimaryFeedback
#define START_ANGLE 0.
#define STOP_ANGLE 180.

/// <summary>
/// Gets the most recent error from the controller and prints it for the user.
/// </summary>
void printError()
{
	// When a function from the C API fails, an error code and message are stored. We can use 
	// Automation1_GetLastError() to get the most recent error code from the controller. Note 
	// that a subsequent error will overwrite the currently stored code and message.
	int32_t lastError = Automation1_GetLastError();

	// We can then use Automation1_GetLastErrorMessage(buffer, bufferLength) to populate a char array with
	// the error message.
	char lastErrorMessage[2048];
	Automation1_GetLastErrorMessage(lastErrorMessage, 2048);

	printf("    error (%d): %s\n", lastError, lastErrorMessage);
}

int32_t main(int32_t argc, char** argv) 
{
  Automation1Controller controller = NULL;
  Automation1StatusConfig statusConfig = NULL;
  double programPosition;
  
  
  printf("Calling Automation1_ConnectWithHost\n");
  if (!Automation1_ConnectWithHost("10.54.160.251", &controller)) {
    printError();
    exit(-1);
  }
  
  printf("Calling Automation1_Command_SetupTaskTargetMode\n");
  if (!Automation1_Command_SetupTaskTargetMode(controller, TASK_INDEX, Automation1TargetMode_Absolute)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_StatusConfig_Create\n");
  if (!Automation1_StatusConfig_Create(&statusConfig)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_StatusConfig_AddAxisStatusItem\n");
  if (!Automation1_StatusConfig_AddAxisStatusItem(statusConfig, AXIS, Automation1AxisStatusItem_ProgramPosition, 0)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Status_GetResults\n");
  if (!Automation1_Status_GetResults(controller, statusConfig, &programPosition, 1)) {
    printError();
    exit(-1);
  }
  printf("Program position=%f\n", programPosition);
  
  printf("Calling Automation1_Command_PsoReset\n");
  if (!Automation1_Command_PsoReset(controller, TASK_INDEX, AXIS)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoDistanceConfigureInputs\n");
  Automation1PsoDistanceInput pulseSrc = PULSE_SRC;
  if (!Automation1_Command_PsoDistanceConfigureInputs(controller, TASK_INDEX, AXIS, &pulseSrc, 1)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoDistanceConfigureFixedDistance\n");
  if (!Automation1_Command_PsoDistanceConfigureFixedDistance(controller, TASK_INDEX, AXIS, 
       std::lround(DEGREES_PER_PULSE * ENCODER_COUNTS_PER_DEGREE))) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoDistanceCounterOn\n");
  if (!Automation1_Command_PsoDistanceCounterOn(controller, TASK_INDEX, AXIS)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoDistanceEventsOn\n");
  if (!Automation1_Command_PsoDistanceEventsOn(controller, TASK_INDEX, AXIS)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoWindowConfigureInput\n");
  if (!Automation1_Command_PsoWindowConfigureInput(controller, TASK_INDEX, AXIS, WINDOW_NUMBER, WINDOW_SRC, 1)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoWindowCCounterSetValue\n");
  if (!Automation1_Command_PsoWindowCounterSetValue(controller, TASK_INDEX, AXIS, WINDOW_NUMBER, 
       std::lround(programPosition*ENCODER_COUNTS_PER_DEGREE))) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoWindowConfigureFixedRange\n");
  if (!Automation1_Command_PsoWindowConfigureFixedRange(controller, TASK_INDEX, AXIS, WINDOW_NUMBER, 
       START_ANGLE*ENCODER_COUNTS_PER_DEGREE, STOP_ANGLE*ENCODER_COUNTS_PER_DEGREE)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoWindowOutputOn\n");
  if (!Automation1_Command_PsoWindowOutputOn(controller, TASK_INDEX, AXIS, WINDOW_NUMBER)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoEventConfigureMask\n");
  if (!Automation1_Command_PsoEventConfigureMask(controller, TASK_INDEX, AXIS, Automation1PsoEventMask_WindowMask)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoWaveformConfigureMode\n");
  if (!Automation1_Command_PsoWaveformConfigureMode(controller, TASK_INDEX, AXIS, Automation1PsoWaveformMode_Pulse)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoWaveformConfigurePulseFixedTotalTime\n");
  if (!Automation1_Command_PsoWaveformConfigurePulseFixedTotalTime(controller, TASK_INDEX, AXIS, 
       std::lround(PULSE_WIDTH_SEC * CLOCK_FREQUENCY))) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoWaveformConfigurePulseFixedOnTime\n");
  if (!Automation1_Command_PsoWaveformConfigurePulseFixedOnTime(controller, TASK_INDEX, AXIS, 
       std::lround(PULSE_WIDTH_SEC * CLOCK_FREQUENCY * DUTY_CYCLE))) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoWaveformConfigurePulseFixedCount\n");
  if (!Automation1_Command_PsoWaveformConfigurePulseFixedCount(controller, TASK_INDEX, AXIS, 1)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoWaveformApplyPulseConfiguration\n");
  if (!Automation1_Command_PsoWaveformApplyPulseConfiguration(controller, TASK_INDEX, AXIS)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoWaveformOn\n");
  if (!Automation1_Command_PsoWaveformOn(controller, TASK_INDEX, AXIS)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoOutputConfigureSource\n");
  if (!Automation1_Command_PsoOutputConfigureSource(controller, TASK_INDEX, AXIS, Automation1PsoOutputSource_Waveform)) {
    printError();
    exit(-1);
  }

  printf("Calling Automation1_Command_PsoOutputConfigureOutput\n");
  if (!Automation1_Command_PsoOutputConfigureOutput(controller, TASK_INDEX, AXIS, Automation1PsoOutputPin_iXL5eAuxiliaryMarkerSingleEnded)) {
    printError();
    exit(-1);
  }

}

