
Automation1CreateController("Automation1", "164.54.104.47", 4, 100, 1000)

# traceError and traceIODriver
#!asynSetTraceMask("Automation1", 0, 0x9)
# traceIOAscii
#!asynSetTraceIOMask("Automation1", 0, 0x1)

Automation1CreateProfile("Automation1", 2000)

dbLoadTemplate "motor.substitutions.automation1"

dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=Auto1:, R=asyn1, PORT=Automation1, ADDR=0, OMAX=256, IMAX=256")

