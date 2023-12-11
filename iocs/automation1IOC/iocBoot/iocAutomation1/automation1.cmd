# Automation1CreateController(
#   const char* portName,    # asyn port name
#   const char* hostName,    # hostname or ip address
#   int numAxes,
#   int movingPollPeriod,    # unit: milliseconds
#   int idlePollPeriod,      # unit: milliseconds
#   int commandExecuteTask,  # default task index: 1
#   int profileMoveTask)     # default task index: 2
Automation1CreateController("Automation1", "127.0.0.1", 4, 100, 1000, 1, 2)

# traceError and traceIODriver
#!asynSetTraceMask("Automation1", 0, 0x9)
# traceIOAscii
#!asynSetTraceIOMask("Automation1", 0, 0x1)

Automation1CreateProfile("Automation1", 2000)

dbLoadTemplate "motor.substitutions.automation1"

dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=Auto1:, R=asyn1, PORT=Automation1, ADDR=0, OMAX=256, IMAX=256")

