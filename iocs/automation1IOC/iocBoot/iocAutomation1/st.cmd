#!../../bin/linux-x86_64/automation1

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/automation1.dbd"
automation1_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"


## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=automation1:")

##

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("automation1:")

# Boot complete
