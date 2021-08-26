#!../../bin/linux-x86_64/automation1

< envPaths

## Register all support components
dbLoadDatabase("../../dbd/automation1.dbd")
automation1_registerRecordDeviceDriver(pdbbase)

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=automation1:")

## Automation1 Support
< automation1.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("automation1:")

##
# This IOC does not use save/restore, so set values of some PVs
dbpf("Auto1:m1.RTRY", "0")
dbpf("Auto1:m1.TWV", "0.1")
dbpf("Auto1:m2.RTRY", "0")
dbpf("Auto1:m2.TWV", "0.1")
dbpf("Auto1:m3.RTRY", "0")
dbpf("Auto1:m3.TWV", "0.1")
dbpf("Auto1:m4.RTRY", "0")
dbpf("Auto1:m4.TWV", "0.1")

# Boot complete
